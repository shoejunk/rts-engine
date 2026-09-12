#pragma once
//==============================================================================
// rts/math/fixed.h - Deterministic 32-bit fixed-point scalar (1.17.14).
//
// The simulation layer of a lockstep RTS must produce bit-identical results on
// every machine, compiler and optimisation level. Floating point cannot promise
// that (x87 excess precision, FMA contraction, fast-math reassociation, libm
// differences), so the sim runs entirely on this type instead.
//
// FORMAT - 1.17.14
//   int32_t storage: 1 sign bit, 17 integer bits, 14 fraction bits. The same
//   layout is also written "signed 18.14"; that convention just folds the sign
//   bit into the whole part instead of listing it separately. Encoding is two's
//   complement, which C++20 mandates for every signed integer anyway.
//     range      [-131072, +131071.99993896484375]
//     resolution 1/16384 = 0.00006103515625
//   The range is asymmetric, as two's complement always is: -kMin overflows and
//   abs(kMin) comes back negative. That is inherent - assert on it, do not try
//   to paper over it.
//   Products and quotients go through int64_t intermediates, so no precision is
//   lost inside an operation - only when the result is stored back to 32 bits.
//
//   kFracBits is the single knob. Every constant, shift and conversion in this
//   header and in angle.h derives from it, so changing the split is a one-line
//   edit (plus re-baselining the determinism fingerprint in the test).
//
// DETERMINISM RULES THIS HEADER FOLLOWS
//   * No floating point survives into runtime code. The only real-to-fixed
//     entry point is Fixed::from_num, which is consteval - it physically cannot
//     be called at runtime, so a stray float can never leak into the sim.
//     to_double() exists for rendering/logging and is strictly one-way.
//   * Every operation is defined behaviour under C++20 rules: signed integers
//     are two's complement, >> on a negative value is an arithmetic shift, and
//     out-of-range narrowing wraps modulo 2^32. Nothing is left
//     implementation-defined, so a debug and a release build agree bit for bit.
//   * Overflow WRAPS (modular); it does not trap or saturate. Wrapping is
//     deterministic, UB is not. Build with RTS_FIXED_CHECKS=1 (the default in
//     debug) to assert on overflow, and use the explicit *_sat helpers where
//     clamping is the behaviour you actually want.
//   * Rounding is round-to-nearest everywhere, ties away from zero. Truncation
//     carries a -0.5 ULP bias that accumulates into visible drift over a long
//     match; round-to-nearest has no systematic bias and costs one add.
//
// WHAT THIS HEADER DOES NOT DO
//   It does not make your simulation deterministic on its own. Still to watch:
//   iteration order of hash containers, pointer-value comparisons, unstable
//   sorts, uninitialised memory, and any use of float in simulation state.
//==============================================================================

#include <bit>
#include <compare>
#include <cstdint>

#if !defined(RTS_FIXED_CHECKS)
#  if defined(NDEBUG)
#    define RTS_FIXED_CHECKS 0
#  else
#    define RTS_FIXED_CHECKS 1
#  endif
#endif

#if RTS_FIXED_CHECKS
#  include <cassert>
#  define RTS_FIXED_ASSERT(x) assert(x)
#else
#  define RTS_FIXED_ASSERT(x) ((void)0)
#endif

namespace rts {

namespace detail {

// Narrow a 64-bit intermediate back to 32-bit storage. The unsigned round-trip
// makes the modular wrap explicit and warning-free; C++20 already defines
// signed narrowing this way, the casts just document the intent.
[[nodiscard]] constexpr std::int32_t narrow(std::int64_t v) noexcept
{
    RTS_FIXED_ASSERT(v >= INT32_MIN && v <= INT32_MAX && "fixed-point overflow");
    return static_cast<std::int32_t>(
        static_cast<std::uint32_t>(static_cast<std::uint64_t>(v)));
}

[[nodiscard]] constexpr std::int32_t saturate(std::int64_t v) noexcept
{
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return static_cast<std::int32_t>(v);
}

// v / 2^shift, rounded to nearest with ties away from zero.
//
// The "- 1 when negative" is what makes ties symmetric: without it an exact
// .5 always rounds toward +infinity, so -(a*b) would not equal (-a)*b. That
// asymmetry is not theoretical - halving a value produces an exact tie for
// every odd raw operand, so a unit mirrored across the map would accumulate a
// different position than its twin. One cmov is a cheap price for f(-v) == -f(v).
[[nodiscard]] constexpr std::int64_t round_shift(std::int64_t v, int shift) noexcept
{
    const std::int64_t half = std::int64_t{ 1 } << (shift - 1);
    return (v + half - (v < 0 ? 1 : 0)) >> shift;
}

// round(n / d), ties away from zero. Caller guarantees d != 0.
// Branch-free on any compiler worth shipping: the select becomes a cmov.
[[nodiscard]] constexpr std::int64_t div_round(std::int64_t n, std::int64_t d) noexcept
{
    const std::int64_t half = (d < 0 ? -d : d) >> 1;
    return (n >= 0) ? (n + half) / d : (n - half) / d;
}

// floor(sqrt(n)) for n < 2^62, plus the remainder needed to round to nearest.
// Restoring shift-and-subtract: no division, no float, and no data-dependent
// branch in the loop body (the mask select compiles to cmov/sbb). The trip
// count does depend on the magnitude of n, which is fine - determinism requires
// identical results, not constant time.
struct SqrtResult
{
    std::uint64_t root;
    std::uint64_t rem;
};

[[nodiscard]] constexpr SqrtResult isqrt64(std::uint64_t n) noexcept
{
    if (n == 0) return { 0, 0 };

    // Start at the highest even power of two <= n, so we skip straight to the
    // first significant digit pair instead of grinding through leading zeros.
    const int msb = 63 - std::countl_zero(n);
    std::uint64_t bit = std::uint64_t{ 1 } << (msb & ~1);
    std::uint64_t res = 0;

    while (bit != 0)
    {
        const std::uint64_t t    = res + bit;
        const std::uint64_t mask = (n >= t) ? ~std::uint64_t{ 0 } : std::uint64_t{ 0 };
        n  -= t & mask;
        res = (res >> 1) + (bit & mask);
        bit >>= 2;
    }
    return { res, n };
}

// rem > root  <=>  n >= (root + 0.5)^2, i.e. the true root rounds up.
[[nodiscard]] constexpr std::uint64_t rounded_root(SqrtResult r) noexcept
{
    return r.root + ((r.rem > r.root) ? 1u : 0u);
}

} // namespace detail

//==============================================================================
// Fixed - 1.17.14 scalar
//==============================================================================
class Fixed
{
public:
    using Raw = std::int32_t;

    static constexpr int kFracBits = 14; // 1.17.14
    static constexpr Raw kRawOne   = Raw{ 1 } << kFracBits;
    static constexpr Raw kRawHalf  = kRawOne >> 1;

    // Largest and smallest whole numbers the format can hold: +/- 2^17.
    static constexpr std::int32_t kMaxWhole =  (Raw{ 1 } << (31 - kFracBits)) - 1;
    static constexpr std::int32_t kMinWhole = -(Raw{ 1 } << (31 - kFracBits));

    // Default-constructs to zero on purpose. An uninitialised simulation value
    // is a desync waiting to happen, and the zeroing costs nothing in practice.
    constexpr Fixed() noexcept : raw_(0) {}

    // Integer -> fixed. Explicit, so accidental int/Fixed mixing never compiles
    // silently and you always see where a conversion happens.
    explicit constexpr Fixed(std::int32_t whole) noexcept
        : raw_(detail::narrow(static_cast<std::int64_t>(whole) << kFracBits))
    {
    }

    [[nodiscard]] static constexpr Fixed from_raw(Raw r) noexcept
    {
        Fixed f;
        f.raw_ = r;
        return f;
    }

    // Compile-time-only conversion from a real literal. consteval is the whole
    // point: it cannot be called at runtime, so no float can reach the sim.
    // Use it for tuning constants - Fixed::from_num(2.5), or 2.5_fx.
    [[nodiscard]] static consteval Fixed from_num(double v) noexcept
    {
        return from_raw(static_cast<Raw>(
            v * static_cast<double>(kRawOne) + (v >= 0.0 ? 0.5 : -0.5)));
    }

    [[nodiscard]] constexpr Raw raw() const noexcept { return raw_; }

    //--- conversions out ------------------------------------------------------
    [[nodiscard]] constexpr std::int32_t to_int() const noexcept // truncate toward zero
    {
        const std::int64_t r = raw_;
        return static_cast<std::int32_t>((r >= 0) ? (r >> kFracBits) : -((-r) >> kFracBits));
    }
    [[nodiscard]] constexpr std::int32_t floor_to_int() const noexcept
    {
        return raw_ >> kFracBits; // arithmetic shift is already a floor
    }
    [[nodiscard]] constexpr std::int32_t ceil_to_int() const noexcept
    {
        return static_cast<std::int32_t>(
            (static_cast<std::int64_t>(raw_) + kRawOne - 1) >> kFracBits);
    }
    [[nodiscard]] constexpr std::int32_t round_to_int() const noexcept
    {
        return static_cast<std::int32_t>(detail::round_shift(raw_, kFracBits));
    }

    // Presentation only - rendering, UI, logging, test output. Never feed the
    // result of this back into simulation state.
    [[nodiscard]] constexpr double to_double() const noexcept
    {
        return static_cast<double>(raw_) / static_cast<double>(kRawOne);
    }

    //--- arithmetic -----------------------------------------------------------
    [[nodiscard]] constexpr Fixed operator-() const noexcept
    {
        return from_raw(detail::narrow(-static_cast<std::int64_t>(raw_)));
    }
    [[nodiscard]] constexpr Fixed operator+() const noexcept { return *this; }

    constexpr Fixed& operator+=(Fixed o) noexcept { *this = *this + o; return *this; }
    constexpr Fixed& operator-=(Fixed o) noexcept { *this = *this - o; return *this; }
    constexpr Fixed& operator*=(Fixed o) noexcept { *this = *this * o; return *this; }
    constexpr Fixed& operator/=(Fixed o) noexcept { *this = *this / o; return *this; }
    constexpr Fixed& operator*=(std::int32_t o) noexcept { *this = *this * o; return *this; }
    constexpr Fixed& operator/=(std::int32_t o) noexcept { *this = *this / o; return *this; }

    friend constexpr Fixed operator+(Fixed a, Fixed b) noexcept
    {
        return from_raw(detail::narrow(static_cast<std::int64_t>(a.raw_) + b.raw_));
    }
    friend constexpr Fixed operator-(Fixed a, Fixed b) noexcept
    {
        return from_raw(detail::narrow(static_cast<std::int64_t>(a.raw_) - b.raw_));
    }

    // The core operation: full 64-bit product, then one rounding step back to
    // the storage format - one imul plus the round_shift (add, shift, and a cmov
    // for the tie direction). Because the product is exact before rounding,
    // there is no double-rounding error to accumulate.
    friend constexpr Fixed operator*(Fixed a, Fixed b) noexcept
    {
        const std::int64_t p = static_cast<std::int64_t>(a.raw_) * b.raw_;
        return from_raw(detail::narrow(detail::round_shift(p, kFracBits)));
    }

    // Scaling by a plain integer needs no shift and no rounding - worth the
    // overload, since this is the common case in simulation code.
    friend constexpr Fixed operator*(Fixed a, std::int32_t b) noexcept
    {
        return from_raw(detail::narrow(static_cast<std::int64_t>(a.raw_) * b));
    }
    friend constexpr Fixed operator*(std::int32_t a, Fixed b) noexcept { return b * a; }

    friend constexpr Fixed operator/(Fixed a, Fixed b) noexcept
    {
        if (b.raw_ == 0)
        {
            RTS_FIXED_ASSERT(false && "fixed-point division by zero");
            // Defined, reproducible fallback. Hardware integer division by zero
            // is UB and traps on x86, which is not a desync you can debug.
            return (a.raw_ == 0) ? Fixed() : from_raw(a.raw_ > 0 ? INT32_MAX : INT32_MIN);
        }
        const std::int64_t n = static_cast<std::int64_t>(a.raw_) << kFracBits;
        return from_raw(detail::narrow(detail::div_round(n, b.raw_)));
    }

    // Dividing by an integer stays in the 32-bit domain: a much cheaper idiv.
    friend constexpr Fixed operator/(Fixed a, std::int32_t b) noexcept
    {
        if (b == 0)
        {
            RTS_FIXED_ASSERT(false && "fixed-point division by zero");
            return (a.raw_ == 0) ? Fixed() : from_raw(a.raw_ > 0 ? INT32_MAX : INT32_MIN);
        }
        return from_raw(detail::narrow(detail::div_round(a.raw_, b)));
    }

    //--- comparison -----------------------------------------------------------
    // The raw ordering is exactly the value ordering for this format, so these are
    // plain integer compares.
    [[nodiscard]] friend constexpr bool operator==(Fixed a, Fixed b) noexcept
    {
        return a.raw_ == b.raw_;
    }
    [[nodiscard]] friend constexpr std::strong_ordering operator<=>(Fixed a, Fixed b) noexcept
    {
        return a.raw_ <=> b.raw_;
    }

private:
    Raw raw_;
};

static_assert(sizeof(Fixed) == 4, "Fixed must stay 4 bytes for replay and network payloads");

//==============================================================================
// Literals: 1.5_fx, 90_fx. consteval, so they cannot smuggle a runtime float in.
//==============================================================================
inline namespace literals {

[[nodiscard]] consteval Fixed operator""_fx(long double v) noexcept
{
    return Fixed::from_num(static_cast<double>(v));
}
[[nodiscard]] consteval Fixed operator""_fx(unsigned long long v) noexcept
{
    return Fixed(static_cast<std::int32_t>(v));
}

} // namespace literals

//==============================================================================
// Constants. Written as raw integers rather than from_num so that not even the
// compile-time path depends on the host's floating point rounding.
//==============================================================================
namespace fx {

inline constexpr Fixed kZero    = Fixed::from_raw(0);
inline constexpr Fixed kOne     = Fixed::from_raw(Fixed::kRawOne);
inline constexpr Fixed kHalf    = Fixed::from_raw(Fixed::kRawHalf);
inline constexpr Fixed kEpsilon = Fixed::from_raw(1);         // 1 ULP
inline constexpr Fixed kMin     = Fixed::from_raw(INT32_MIN); // -131072
inline constexpr Fixed kMax     = Fixed::from_raw(INT32_MAX); // ~+131071.99994

// Irrationals are held at Q2.30 and shifted into place, so they follow
// kFracBits automatically and never depend on host floating point - not even
// at compile time.
namespace detail {

inline constexpr std::int64_t kPiQ30    = 3373259426; // pi       * 2^30
inline constexpr std::int64_t kTauQ30   = 6746518852; // 2*pi     * 2^30
inline constexpr std::int64_t kSqrt2Q30 = 1518500250; // sqrt(2)  * 2^30

// Valid for kFracBits < 30, which every sane split satisfies.
[[nodiscard]] constexpr Fixed from_q30(std::int64_t q30) noexcept
{
    return Fixed::from_raw(rts::detail::narrow(
        rts::detail::round_shift(q30, 30 - Fixed::kFracBits)));
}

} // namespace detail

inline constexpr Fixed kPi    = detail::from_q30(detail::kPiQ30);
inline constexpr Fixed kTwoPi = detail::from_q30(detail::kTauQ30);
inline constexpr Fixed kSqrt2 = detail::from_q30(detail::kSqrt2Q30);

} // namespace fx

//==============================================================================
// Free functions
//==============================================================================

[[nodiscard]] constexpr Fixed abs(Fixed a) noexcept
{
    return (a.raw() < 0) ? -a : a;
}

[[nodiscard]] constexpr std::int32_t sign(Fixed a) noexcept
{
    return static_cast<std::int32_t>(a.raw() > 0) - static_cast<std::int32_t>(a.raw() < 0);
}

[[nodiscard]] constexpr Fixed min(Fixed a, Fixed b) noexcept { return (a.raw() < b.raw()) ? a : b; }
[[nodiscard]] constexpr Fixed max(Fixed a, Fixed b) noexcept { return (a.raw() > b.raw()) ? a : b; }

[[nodiscard]] constexpr Fixed clamp(Fixed v, Fixed lo, Fixed hi) noexcept
{
    RTS_FIXED_ASSERT(lo.raw() <= hi.raw());
    return min(max(v, lo), hi);
}

[[nodiscard]] constexpr Fixed floor(Fixed a) noexcept
{
    return Fixed::from_raw(a.raw() & ~(Fixed::kRawOne - 1));
}
[[nodiscard]] constexpr Fixed ceil(Fixed a) noexcept
{
    return Fixed::from_raw(detail::narrow(
        (static_cast<std::int64_t>(a.raw()) + Fixed::kRawOne - 1)
        & ~static_cast<std::int64_t>(Fixed::kRawOne - 1)));
}
[[nodiscard]] constexpr Fixed round(Fixed a) noexcept
{
    return Fixed::from_raw(detail::narrow(
        detail::round_shift(a.raw(), Fixed::kFracBits) << Fixed::kFracBits));
}

// Always in [0, 1): the fractional part relative to floor(), not to zero.
[[nodiscard]] constexpr Fixed frac(Fixed a) noexcept
{
    return Fixed::from_raw(a.raw() & (Fixed::kRawOne - 1));
}

// a + (b - a) * t, carried in 64 bits so the intermediate cannot overflow even
// when a and b sit at opposite ends of the range. Exact at t == 0 and t == 1.
[[nodiscard]] constexpr Fixed lerp(Fixed a, Fixed b, Fixed t) noexcept
{
    const std::int64_t d = static_cast<std::int64_t>(b.raw()) - a.raw();
    return Fixed::from_raw(detail::narrow(
        a.raw() + detail::round_shift(d * t.raw(), Fixed::kFracBits)));
}

// (a * b) / c with a single 64-bit intermediate and one rounding step. Use this
// rather than a * b / c whenever the product might leave the representable
// range - the intermediate here never does, so the result is exact.
[[nodiscard]] constexpr Fixed mul_div(Fixed a, Fixed b, Fixed c) noexcept
{
    if (c.raw() == 0)
    {
        RTS_FIXED_ASSERT(false && "fixed-point division by zero");
        return Fixed();
    }
    const std::int64_t p = static_cast<std::int64_t>(a.raw()) * b.raw();
    return Fixed::from_raw(detail::narrow(detail::div_round(p, c.raw())));
}

//--- saturating variants ------------------------------------------------------
// Clamp instead of wrap. Costs a compare and a select; reach for these at the
// boundaries where a wrap would be absurd (health, resource totals, timers).

[[nodiscard]] constexpr Fixed add_sat(Fixed a, Fixed b) noexcept
{
    return Fixed::from_raw(detail::saturate(static_cast<std::int64_t>(a.raw()) + b.raw()));
}
[[nodiscard]] constexpr Fixed sub_sat(Fixed a, Fixed b) noexcept
{
    return Fixed::from_raw(detail::saturate(static_cast<std::int64_t>(a.raw()) - b.raw()));
}
[[nodiscard]] constexpr Fixed mul_sat(Fixed a, Fixed b) noexcept
{
    const std::int64_t p = static_cast<std::int64_t>(a.raw()) * b.raw();
    return Fixed::from_raw(detail::saturate(detail::round_shift(p, Fixed::kFracBits)));
}

//--- roots --------------------------------------------------------------------
// sqrt(x) == isqrt(raw << f), since sqrt(raw / 2^f) * 2^f == sqrt(raw * 2^f).
// At 14 fraction bits the shifted operand peaks just under 2^45 and its root
// just over 2^22, so this cannot overflow for any in-range input. Result is
// round-to-nearest.
[[nodiscard]] constexpr Fixed sqrt(Fixed a) noexcept
{
    RTS_FIXED_ASSERT(a.raw() >= 0 && "sqrt of a negative fixed-point value");
    if (a.raw() <= 0) return Fixed();

    const auto r = detail::isqrt64(static_cast<std::uint64_t>(a.raw()) << Fixed::kFracBits);
    return Fixed::from_raw(static_cast<std::int32_t>(detail::rounded_root(r)));
}

// Length of a 2D vector without ever forming x*x + y*y in 32 bits, which would
// overflow past ~362 units. Everything stays in the 64-bit domain until the
// final root, so this is exact for the whole input range even though the
// *result* still has to fit the format. Hot path for range checks and steering.
[[nodiscard]] constexpr Fixed hypot(Fixed x, Fixed y) noexcept
{
    const std::int64_t xx = static_cast<std::int64_t>(x.raw()) * x.raw();
    const std::int64_t yy = static_cast<std::int64_t>(y.raw()) * y.raw();
    const auto r = detail::isqrt64(static_cast<std::uint64_t>(xx + yy));
    return Fixed::from_raw(detail::narrow(static_cast<std::int64_t>(detail::rounded_root(r))));
}

// Squared length, kept in 64 bits. Prefer this for "is it in range" tests: it
// skips the root entirely and never overflows.
[[nodiscard]] constexpr std::int64_t length_sq_raw(Fixed x, Fixed y) noexcept
{
    return static_cast<std::int64_t>(x.raw()) * x.raw()
         + static_cast<std::int64_t>(y.raw()) * y.raw();
}

} // namespace rts
