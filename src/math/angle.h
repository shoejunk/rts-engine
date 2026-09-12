#pragma once
//==============================================================================
// rts/math/angle.h - Deterministic angles and trigonometry.
//
// WHY BINARY ANGLES INSTEAD OF RADIANS
//   An angle stored as a uint32 where the full turn is exactly 2^32 has three
//   properties that matter more than anything else in a lockstep simulation:
//
//     1. Wrapping is free and exact. Adding a turn rate every tick for an hour
//        never drifts, because uint32 overflow *is* the modulo. Storing radians
//        in the fixed format instead means wrapping through an inexact 2*pi,
//        and that error compounds every single tick.
//     2. The quadrant is the top two bits, so argument reduction is a shift and
//        a mask rather than a division.
//     3. Resolution is 2^-32 of a turn (0.00000008 degrees) for the same four
//        bytes a fixed-point radian would cost, and the cardinal angles - 0,
//        90, 180, 270 - are representable exactly.
//
//   The one thing BAM does not give you is exact whole degrees: 360 is not a
//   power of two, so only multiples of 45 degrees land precisely. Everything
//   else is within half a BAM unit (2e-8 degrees), which is four orders of
//   magnitude finer than the values sin and cos produce - but it does
//   mean that adding from_degrees(1) 360 times is not exactly a full turn.
//   Accumulate in raw BAM, not in degrees.
//
//   Conversions to and from degrees and turns are provided at the edges; the
//   simulation itself should carry Angle values end to end.
//
// ACCURACY (measured, see tests/fixed_tests.cpp)
//   sin/cos  <= 0.51 ULP of the storage format, exact at 0/90/180/270 degrees.
//   atan2    <= 1841 BAM = 0.00016 degrees, and exact at all eight cardinals.
//
// IMPLEMENTATION
//   Odd minimax polynomials evaluated by Horner in Q2.30 with int64 products.
//   No tables: 8 KB of sine table costs more in cache misses on a sim touching
//   thousands of units per tick than the four multiplies it saves. No CORDIC
//   either - that trades these few multiplies for ~30 dependent iterations.
//==============================================================================

#include "fixed.h"

namespace rts {

namespace detail {

// Working format for trig is Q2.30: one bit of headroom above 1.0 for
// coefficients up to pi/2, and 30 fraction bits, which leaves 16 bits of guard
// below the 1.17.14 output so the final rounding is the only visible error.
inline constexpr std::int64_t kOneQ30  = std::int64_t{ 1 } << 30;
inline constexpr std::int64_t kHalfQ30 = std::int64_t{ 1 } << 29;

[[nodiscard]] constexpr std::int64_t mul_q30(std::int64_t a, std::int64_t b) noexcept
{
    return (a * b + kHalfQ30) >> 30; // arithmetic shift: floor, per C++20
}

//--- sin, degree-7 odd minimax on the quarter turn -----------------------------
// S(x) = sin(pi*x/2) for x in [0,1], fitted as x*(c1 + c3*x^2 + c5*x^4 + c7*x^6)
// under the constraint S(1) == 1 exactly, so sin(90 degrees) is exactly 1.0
// rather than one ULP short. Degree 7 leaves 0.015 ULP of approximation error
// at 14 fraction bits - far under the 0.5 ULP quantisation floor, so a degree-9
// fit would buy nothing observable and cost another multiply.
inline constexpr std::int64_t kSinC3 = -693526079;
inline constexpr std::int64_t kSinC5 = 85298167;
inline constexpr std::int64_t kSinC7 = -4654809;
inline constexpr std::int64_t kSinC1 = kOneQ30 - kSinC3 - kSinC5 - kSinC7; // 1686624545

static_assert(kSinC1 + kSinC3 + kSinC5 + kSinC7 == kOneQ30,
              "sin coefficients must sum to 1.0 so sin(90 degrees) is exact");

// x is the position within a quarter turn, Q30 in [0, 2^30]. Returns Q30.
[[nodiscard]] constexpr std::int64_t sin_q30(std::int64_t x) noexcept
{
    const std::int64_t u = mul_q30(x, x);
    std::int64_t p = kSinC7;
    p = mul_q30(p, u) + kSinC5;
    p = mul_q30(p, u) + kSinC3;
    p = mul_q30(p, u) + kSinC1;
    return mul_q30(p, x);
}

//--- atan, degree-11 odd minimax on [0,1] -------------------------------------
// T(z) = atan(z)/(2*pi) in turns, constrained so T(1) == 1/8 exactly, which is
// what makes atan2 land precisely on 45/135/225/315 degrees as well as on the
// axes. Degree 11 gives 4.3e-7 turns; atan converges slowly on [0,1], which is
// why it needs more terms than sin does.
inline constexpr std::int64_t kAtanB3  = -56845241;
inline constexpr std::int64_t kAtanB5  = 33092753;
inline constexpr std::int64_t kAtanB7  = -19940571;
inline constexpr std::int64_t kAtanB9  = 9041004;
inline constexpr std::int64_t kAtanB11 = -2017754;
inline constexpr std::int64_t kAtanB1  =
    (kOneQ30 >> 3) - kAtanB3 - kAtanB5 - kAtanB7 - kAtanB9 - kAtanB11; // 170887537

static_assert(kAtanB1 + kAtanB3 + kAtanB5 + kAtanB7 + kAtanB9 + kAtanB11 == (kOneQ30 >> 3),
              "atan coefficients must sum to 1/8 turn so atan2(1,1) is exactly 45 degrees");

// z is a ratio, Q30 in [0, 2^30]. Returns turns, Q30 in [0, 2^30/8].
[[nodiscard]] constexpr std::int64_t atan_turns_q30(std::int64_t z) noexcept
{
    const std::int64_t u = mul_q30(z, z);
    std::int64_t p = kAtanB11;
    p = mul_q30(p, u) + kAtanB9;
    p = mul_q30(p, u) + kAtanB7;
    p = mul_q30(p, u) + kAtanB5;
    p = mul_q30(p, u) + kAtanB3;
    p = mul_q30(p, u) + kAtanB1;
    return mul_q30(p, z);
}

} // namespace detail

//==============================================================================
// Angle - binary angle measurement, 2^32 raw units per full turn
//==============================================================================
class Angle
{
public:
    using Raw = std::uint32_t;

    static constexpr Raw kRawQuarterTurn = 0x40000000u;
    static constexpr Raw kRawHalfTurn    = 0x80000000u;

    constexpr Angle() noexcept : raw_(0) {}

    [[nodiscard]] static constexpr Angle from_raw(Raw r) noexcept
    {
        Angle a;
        a.raw_ = r;
        return a;
    }

    // Whole degrees. Exact only for multiples of 45 (the fractions of a turn
    // that are powers of two); everything else rounds to the nearest BAM unit,
    // which is 2e-8 degrees. Input is reduced modulo 360 first, so large and
    // negative values are well defined rather than overflowing.
    [[nodiscard]] static constexpr Angle from_degrees(std::int32_t deg) noexcept
    {
        const std::int64_t d = deg % 360;
        return from_raw(static_cast<Raw>(
            detail::div_round(d * (std::int64_t{ 1 } << 32), 360)));
    }

    // Compile-time only, same reasoning as Fixed::from_num.
    [[nodiscard]] static consteval Angle from_degrees(double deg) noexcept
    {
        const double t = deg / 360.0;
        return from_raw(static_cast<Raw>(static_cast<std::int64_t>(
            t * 4294967296.0 + (t >= 0.0 ? 0.5 : -0.5))));
    }

    // Turns as a Fixed: 0.25 is a quarter turn. Values outside [0,1) wrap.
    [[nodiscard]] static constexpr Angle from_turns(Fixed turns) noexcept
    {
        return from_raw(static_cast<Raw>(static_cast<std::uint64_t>(
            static_cast<std::int64_t>(turns.raw()) << (32 - Fixed::kFracBits))));
    }

    [[nodiscard]] constexpr Raw raw() const noexcept { return raw_; }

    // Reinterpreted as signed, an Angle is the shortest offset from zero, in
    // [-0.5, +0.5) turns. This is the form you want for steering errors.
    [[nodiscard]] constexpr std::int32_t signed_raw() const noexcept
    {
        return static_cast<std::int32_t>(raw_);
    }

    [[nodiscard]] constexpr Fixed to_turns() const noexcept // [0, 1)
    {
        return Fixed::from_raw(static_cast<std::int32_t>(raw_ >> (32 - Fixed::kFracBits)));
    }
    [[nodiscard]] constexpr Fixed to_signed_turns() const noexcept // [-0.5, 0.5)
    {
        return Fixed::from_raw(signed_raw() >> (32 - Fixed::kFracBits));
    }
    [[nodiscard]] constexpr Fixed to_degrees() const noexcept // [0, 360)
    {
        // raw * 360 / 2^32, carried into the fixed format: (raw * 360 * one) >> 32.
        constexpr std::uint64_t kScale = 360ull * static_cast<std::uint64_t>(Fixed::kRawOne);
        return Fixed::from_raw(static_cast<std::int32_t>(
            ((static_cast<std::uint64_t>(raw_) * kScale) + (1ull << 31)) >> 32));
    }

    //--- arithmetic: every one of these wraps exactly, for free ----------------
    [[nodiscard]] constexpr Angle operator-() const noexcept { return from_raw(0u - raw_); }
    [[nodiscard]] constexpr Angle operator+() const noexcept { return *this; }

    friend constexpr Angle operator+(Angle a, Angle b) noexcept { return from_raw(a.raw_ + b.raw_); }
    friend constexpr Angle operator-(Angle a, Angle b) noexcept { return from_raw(a.raw_ - b.raw_); }
    friend constexpr Angle operator*(Angle a, std::int32_t s) noexcept
    {
        return from_raw(a.raw_ * static_cast<Raw>(s));
    }
    friend constexpr Angle operator*(std::int32_t s, Angle a) noexcept { return a * s; }

    constexpr Angle& operator+=(Angle o) noexcept { raw_ += o.raw_; return *this; }
    constexpr Angle& operator-=(Angle o) noexcept { raw_ -= o.raw_; return *this; }

    // Only equality. Ordering is not defined on a circle - use signed_raw() on
    // a difference when you need "which way and how far".
    [[nodiscard]] friend constexpr bool operator==(Angle a, Angle b) noexcept
    {
        return a.raw_ == b.raw_;
    }

private:
    Raw raw_;
};

static_assert(sizeof(Angle) == 4, "Angle must stay 4 bytes for replay and network payloads");

namespace ang {

inline constexpr Angle kZero        = Angle::from_raw(0);
inline constexpr Angle kQuarterTurn = Angle::from_raw(Angle::kRawQuarterTurn); // 90 degrees
inline constexpr Angle kHalfTurn    = Angle::from_raw(Angle::kRawHalfTurn);    // 180 degrees

} // namespace ang

//==============================================================================
// Trigonometry
//==============================================================================

// Quadrant folding: the top two bits pick the quadrant, the low 30 are the
// position within it. Mirror on odd quadrants, negate on the bottom half - two
// predicated operations, no branches worth the name.
[[nodiscard]] constexpr Fixed sin(Angle a) noexcept
{
    const std::uint32_t r    = a.raw();
    const std::uint32_t quad = r >> 30;

    std::int64_t x = static_cast<std::int64_t>(r & 0x3FFFFFFFu);
    if (quad & 1u) x = detail::kOneQ30 - x;

    std::int64_t s = detail::sin_q30(x);
    if (quad & 2u) s = -s;

    // Q2.30 -> the storage format. round_shift rather than a plain biased shift:
    // ties away from zero keeps sin(-t) == -sin(t) exactly, which a half-up
    // shift breaks whenever the Q30 result lands precisely on a half ULP.
    return Fixed::from_raw(static_cast<std::int32_t>(
        detail::round_shift(s, 30 - Fixed::kFracBits)));
}

[[nodiscard]] constexpr Fixed cos(Angle a) noexcept
{
    return sin(a + ang::kQuarterTurn);
}

// Both at once - what you actually need to build a rotation.
struct SinCos
{
    Fixed sin;
    Fixed cos;
};

[[nodiscard]] constexpr SinCos sin_cos(Angle a) noexcept
{
    return { sin(a), cos(a) };
}

// Full four-quadrant arctangent. Takes Fixed for convenience, but only the
// ratio matters, so any consistent units work.
//
// Folds the input into the first octant so the polynomial only ever sees
// z in [0,1], then rebuilds the angle from the swap and sign flags. One 64-bit
// division and six multiplies.
[[nodiscard]] constexpr Angle atan2(Fixed y, Fixed x) noexcept
{
    const std::int64_t yr = y.raw();
    const std::int64_t xr = x.raw();
    const std::int64_t ay = (yr < 0) ? -yr : yr;
    const std::int64_t ax = (xr < 0) ? -xr : xr;

    if ((ax | ay) == 0) return ang::kZero; // atan2(0,0): defined as 0 here

    const bool swap = (ay > ax);
    const std::int64_t num = swap ? ax : ay;
    const std::int64_t den = swap ? ay : ax;

    // Q30 ratio in [0,1], rounded. num <= 2^31 so num << 30 stays inside int64.
    const std::int64_t z = ((num << 30) + (den >> 1)) / den;

    // Q30 turns -> BAM is a shift by 2, since BAM is Q32 turns.
    std::uint32_t t = static_cast<std::uint32_t>(detail::atan_turns_q30(z) << 2);

    if (swap)   t = Angle::kRawQuarterTurn - t; // reflect about 45 degrees
    if (xr < 0) t = Angle::kRawHalfTurn - t;    // reflect into the left half
    if (yr < 0) t = 0u - t;                     // mirror below the axis

    return Angle::from_raw(t);
}

//==============================================================================
// Steering helpers
//==============================================================================

// Shortest signed path from a to b, in [-0.5, +0.5) turns. The subtraction
// wraps, so this is correct across the 0/360 seam with no special case at all.
[[nodiscard]] constexpr std::int32_t shortest_delta_raw(Angle from, Angle to) noexcept
{
    return static_cast<std::int32_t>(to.raw() - from.raw());
}

[[nodiscard]] constexpr Fixed shortest_delta_turns(Angle from, Angle to) noexcept
{
    return Fixed::from_raw(shortest_delta_raw(from, to) >> (32 - Fixed::kFracBits));
}

// Turn from `current` toward `target` by at most `max_step`, taking the short
// way round and snapping exactly onto the target once within reach. A unit
// exactly 180 degrees from its target turns negative - an arbitrary but fixed
// tie-break, which is all determinism asks for.
[[nodiscard]] constexpr Angle rotate_towards(Angle current, Angle target, Angle max_step) noexcept
{
    const std::uint32_t step = max_step.raw();
    if (step >= Angle::kRawHalfTurn) return target; // can reach anywhere

    const std::int32_t delta = shortest_delta_raw(current, target);
    const std::int32_t s     = static_cast<std::int32_t>(step);

    if (delta > s)  return Angle::from_raw(current.raw() + step);
    if (delta < -s) return Angle::from_raw(current.raw() - step);
    return target;
}

} // namespace rts
