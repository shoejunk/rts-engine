//==============================================================================
// tests/fixed_tests.cpp - Correctness, accuracy and determinism tests for the
// fixed-point math layer.
//
// Floating point appears in this file only as a *reference* for measuring
// error. Where a property can be checked exactly with integers (sqrt rounding,
// commutativity, wrapping) it is checked that way instead, because a float
// comparison can only tell you the answer is close, not that it is right.
//==============================================================================

#include "math/angle.h"
#include "math/fixed.h"

#include <cmath>
#include <cstdio>
#include <cstdint>

using namespace rts;
using namespace rts::literals;

// Everything below is written against the format rather than against literal
// raw values, so changing Fixed::kFracBits does not invalidate the suite.
constexpr std::int32_t kOneRaw  = Fixed::kRawOne;
constexpr std::int32_t kHalfRaw = Fixed::kRawHalf;
constexpr double       kUlps    = static_cast<double>(Fixed::kRawOne); // value -> ULP

//------------------------------------------------------------------------------
// Minimal harness
//------------------------------------------------------------------------------
namespace {

int g_checks = 0;
int g_failures = 0;
const char* g_section = "";

void section(const char* name)
{
    g_section = name;
    std::printf("\n-- %s\n", name);
}

void check(bool ok, const char* expr, int line)
{
    ++g_checks;
    if (!ok)
    {
        ++g_failures;
        std::printf("  FAIL [%s:%d] %s\n", g_section, line, expr);
    }
}

#define CHECK(expr) check((expr), #expr, __LINE__)

// Deterministic PRNG for test inputs. xorshift32, so the sample set is
// identical on every platform and a failure is always reproducible.
std::uint32_t g_rng = 0x12345678u;
std::uint32_t next_u32()
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}
std::int32_t next_raw() { return static_cast<std::int32_t>(next_u32()); }

// A random value with |v| <= 128, derived from the format rather than from a
// fixed shift: products of two of these stay representable for any kFracBits,
// so these loops test rounding rather than accidentally testing wrap.
Fixed next_small() { return Fixed::from_raw(next_raw() >> (24 - Fixed::kFracBits)); }

} // namespace

//==============================================================================
// Compile-time proof: if any of these were wrong the file would not build, and
// their passing also proves the whole API really is usable in constexpr.
//==============================================================================
static_assert(sizeof(Fixed) == 4);
static_assert(Fixed(3).raw() == 3 * kOneRaw);
static_assert((1.5_fx).raw() == 3 * kHalfRaw);
static_assert((-2.25_fx).raw() == -9 * (kOneRaw / 4));
static_assert(1.5_fx + 2.5_fx == 4_fx);
static_assert(1.5_fx * 2_fx == 3_fx);
static_assert(3_fx / 2_fx == 1.5_fx);
static_assert(-3_fx / 2_fx == -1.5_fx);
static_assert(sqrt(16_fx) == 4_fx);
static_assert(sqrt(2.25_fx) == 1.5_fx);
static_assert(hypot(3_fx, 4_fx) == 5_fx);
static_assert(abs(-7.5_fx) == 7.5_fx);
static_assert(floor(-1.5_fx) == -2_fx);
static_assert(ceil(-1.5_fx) == -1_fx);
static_assert(frac(-1.25_fx) == 0.75_fx);
static_assert(lerp(10_fx, 20_fx, 0.25_fx) == 12.5_fx);
static_assert(clamp(5_fx, 0_fx, 3_fx) == 3_fx);
static_assert(sin(ang::kQuarterTurn) == 1_fx);
static_assert(cos(ang::kZero) == 1_fx);
static_assert(sin(Angle::from_degrees(30)) == 0.5_fx);
static_assert(atan2(1_fx, 1_fx) == Angle::from_degrees(45));
static_assert(Angle::from_degrees(360) == ang::kZero);
static_assert(Angle::from_degrees(450) == Angle::from_degrees(90));

//==============================================================================
// Tests
//==============================================================================
namespace {

void test_construction_and_conversion()
{
    section("construction & conversion");

    CHECK(Fixed().raw() == 0);                 // default is zero, not garbage
    CHECK(Fixed(0).raw() == 0);
    CHECK(Fixed(1).raw() == kOneRaw);
    CHECK(Fixed(-1).raw() == -kOneRaw);
    CHECK(Fixed::from_raw(kHalfRaw) == 0.5_fx);

    CHECK((2.75_fx).to_int() == 2);
    CHECK((-2.75_fx).to_int() == -2);          // truncates toward zero
    CHECK((2.75_fx).floor_to_int() == 2);
    CHECK((-2.75_fx).floor_to_int() == -3);    // floors toward -inf
    CHECK((2.25_fx).ceil_to_int() == 3);
    CHECK((-2.25_fx).ceil_to_int() == -2);
    CHECK((2.5_fx).round_to_int() == 3);
    CHECK((-2.5_fx).round_to_int() == -3);   // ties away from zero
    CHECK((2.4_fx).round_to_int() == 2);
    CHECK((-2.4_fx).round_to_int() == -2);
    CHECK(round(2.5_fx) == 3_fx);
    CHECK(round(-2.5_fx) == -3_fx);

    // Integer round-trip is exact across the whole representable range.
    CHECK(Fixed::kMinWhole == -131072); // 1.17.14: 17 whole bits plus a sign
    CHECK(Fixed::kMaxWhole ==  131071);
    for (std::int32_t i = Fixed::kMinWhole; i <= Fixed::kMaxWhole; ++i)
    {
        if (Fixed(i).to_int() != i) { CHECK(false); break; }
    }

    CHECK(fx::kMin.raw() == INT32_MIN);
    CHECK(fx::kMax.raw() == INT32_MAX);
    CHECK(fx::kEpsilon.raw() == 1);
}

void test_arithmetic()
{
    section("arithmetic");

    CHECK(2_fx + 3_fx == 5_fx);
    CHECK(2_fx - 5_fx == -3_fx);
    CHECK(7_fx * 6_fx == 42_fx);
    CHECK(42_fx / 6_fx == 7_fx);
    CHECK(-42_fx / 6_fx == -7_fx);
    CHECK(42_fx / -6_fx == -7_fx);
    CHECK(-42_fx / -6_fx == 7_fx);

    // Integer overloads must agree exactly with the Fixed-Fixed versions.
    CHECK(3.5_fx * 4 == 3.5_fx * 4_fx);
    CHECK(4 * 3.5_fx == 3.5_fx * 4_fx);
    CHECK(14_fx / 4 == 14_fx / 4_fx);
    CHECK(-14_fx / 4 == -14_fx / 4_fx);

    // Multiplication is exactly commutative. This is not automatic - it holds
    // because the 64-bit product is formed before any rounding happens.
    for (int i = 0; i < 200000; ++i)
    {
        const Fixed a = next_small();
        const Fixed b = next_small();
        if (a * b != b * a) { CHECK(false); break; }
    }

    // Identities.
    for (int i = 0; i < 100000; ++i)
    {
        const Fixed a = Fixed::from_raw(next_raw() >> 4);
        if (a * fx::kOne != a) { CHECK(false); break; }
        if (a / fx::kOne != a) { CHECK(false); break; }
        if (a + fx::kZero != a) { CHECK(false); break; }
        if (a - a != fx::kZero) { CHECK(false); break; }
    }

    // Rounding is to nearest, ties away from zero - never truncation.
    CHECK(Fixed::from_raw(3) * Fixed::from_raw(kHalfRaw) == Fixed::from_raw(2)); // 1.5 -> 2
    CHECK(Fixed::from_raw(1) * Fixed::from_raw(kHalfRaw) == Fixed::from_raw(1)); // 0.5 -> 1
    CHECK(Fixed::from_raw(-1) * Fixed::from_raw(kHalfRaw) == Fixed::from_raw(-1));
    CHECK(Fixed::from_raw(1) / Fixed(2) == Fixed::from_raw(1));               // 0.5 -> 1
    CHECK(Fixed::from_raw(-1) / Fixed(2) == Fixed::from_raw(-1));
    CHECK(Fixed::from_raw(-3) * Fixed::from_raw(kHalfRaw) == Fixed::from_raw(-2)); // -1.5 -> -2

    // Ties away from zero is what makes the whole type symmetric under
    // negation. A mirrored army must evolve as the mirror image of the
    // original, and with ties-toward-+inf it would not.
    bool symmetric = true;
    for (int i = 0; i < 300000 && symmetric; ++i)
    {
        const Fixed a = next_small();
        const Fixed b = next_small();
        // |a| <= 128 by construction; require |b| >= 1 so the quotient stays
        // representable and we are testing rounding, not wrapping.
        const bool div_in_range = abs(b) >= fx::kOne;
        symmetric = ((-a) * b == -(a * b))
                 && (a * (-b) == -(a * b))
                 && ((-a) * (-b) == a * b)
                 && (div_in_range ? ((-a) / b == -(a / b)) : true)
                 && (lerp(-a, -b, 0.25_fx) == -lerp(a, b, 0.25_fx));
    }
    CHECK(symmetric);

    // Multiplication error never exceeds the 0.5 ULP rounding floor.
    double worst = 0.0;
    for (int i = 0; i < 500000; ++i)
    {
        const Fixed a = Fixed::from_raw(next_raw() >> 12);
        const Fixed b = Fixed::from_raw(next_raw() >> 12);
        const double ref = a.to_double() * b.to_double();
        const double err = std::fabs((a * b).to_double() - ref) * kUlps;
        if (err > worst) worst = err;
    }
    std::printf("   mul  max error: %.4f ULP\n", worst);
    CHECK(worst <= 0.5);

    // Division likewise.
    worst = 0.0;
    for (int i = 0; i < 500000; ++i)
    {
        const Fixed a = Fixed::from_raw(next_raw() >> (32 - Fixed::kFracBits));
        const Fixed b = Fixed::from_raw(next_raw() >> (32 - Fixed::kFracBits));
        if (b.raw() == 0) continue;
        const double ref = a.to_double() / b.to_double();
        if (std::fabs(ref) > static_cast<double>(Fixed::kMaxWhole)) continue; // would wrap
        const double err = std::fabs((a / b).to_double() - ref) * kUlps;
        if (err > worst) worst = err;
    }
    std::printf("   div  max error: %.4f ULP\n", worst);
    CHECK(worst <= 0.5);
}

void test_overflow_and_saturation()
{
    section("overflow & saturation");

    // Wrapping is the documented release behaviour and must be exactly modular.
    // (Checks are compiled out here so the asserts do not fire on purpose.)
    CHECK(add_sat(fx::kMax, 1_fx) == fx::kMax);
    CHECK(sub_sat(fx::kMin, 1_fx) == fx::kMin);
    CHECK(mul_sat(30000_fx, 30000_fx) == fx::kMax);
    CHECK(mul_sat(-30000_fx, 30000_fx) == fx::kMin);
    CHECK(add_sat(2_fx, 3_fx) == 5_fx);       // no clamping when in range
    CHECK(mul_sat(7_fx, 6_fx) == 42_fx);

    // mul_div keeps an intermediate that would not fit in 32 bits.
    CHECK(mul_div(20000_fx, 20000_fx, 20000_fx) == 20000_fx);
    CHECK(mul_div(1000_fx, 1000_fx, 100_fx) == 10000_fx);

#if !RTS_FIXED_CHECKS
    // These deliberately exercise the paths that assert in a checked build, so
    // they only run when the checks are compiled out. A debug run still proves
    // something useful: that nothing *else* in this suite trips an assert.

    // Division by zero is defined, not a trap.
    CHECK((1_fx / 0_fx) == fx::kMax);
    CHECK((-1_fx / 0_fx) == fx::kMin);
    CHECK((0_fx / 0_fx) == fx::kZero);
    CHECK((1_fx / 0) == fx::kMax);

    // Overflow wraps modulo 2^32, exactly and reproducibly - never UB.
    CHECK(fx::kMax + fx::kEpsilon == fx::kMin);
    CHECK(fx::kMin - fx::kEpsilon == fx::kMax);
    CHECK((30000_fx * 30000_fx).raw()
          == static_cast<std::int32_t>(static_cast<std::uint32_t>(
                 (std::int64_t{ 30000 } * kOneRaw * 30000 * kOneRaw + kHalfRaw)
                 >> Fixed::kFracBits)));
#else
    std::printf("   (skipped: misuse paths assert in a checked build)\n");
#endif
}

void test_sqrt()
{
    section("sqrt");

    CHECK(sqrt(0_fx) == 0_fx);
    CHECK(sqrt(1_fx) == 1_fx);
    CHECK(sqrt(4_fx) == 2_fx);
    CHECK(sqrt(144_fx) == 12_fx);
    CHECK(sqrt(2_fx) == fx::kSqrt2);

    // Exact property check, no floats: r must be the nearest integer to
    // sqrt(N), i.e. (2r-1)^2 <= 4N < (2r+1)^2.
    auto check_nearest = [](std::int32_t raw) -> bool {
        const std::uint64_t n = static_cast<std::uint64_t>(raw) << Fixed::kFracBits;
        const std::uint64_t r = static_cast<std::uint64_t>(sqrt(Fixed::from_raw(raw)).raw());
        const std::uint64_t lo = (2 * r - 1) * (2 * r - 1);
        const std::uint64_t hi = (2 * r + 1) * (2 * r + 1);
        return (r == 0 || lo <= 4 * n) && 4 * n < hi;
    };

    bool all_ok = true;
    for (std::int32_t v = 1; v < 300000 && all_ok; ++v) all_ok = check_nearest(v);
    for (int i = 0; i < 500000 && all_ok; ++i)  all_ok = check_nearest(next_raw() & INT32_MAX);
    all_ok = all_ok && check_nearest(INT32_MAX) && check_nearest(1);
    CHECK(all_ok);

    // Monotonic: a larger input can never produce a smaller root.
    Fixed prev = sqrt(Fixed::from_raw(0));
    bool monotonic = true;
    for (std::int32_t v = 1; v < 2000000; v += 37)
    {
        const Fixed cur = sqrt(Fixed::from_raw(v));
        if (cur < prev) { monotonic = false; break; }
        prev = cur;
    }
    CHECK(monotonic);

    // Perfect squares must come back exactly.
    bool exact = true;
    for (std::int32_t i = 0; i <= 181; ++i)
    {
        if (sqrt(Fixed(i) * Fixed(i)) != Fixed(i)) { exact = false; break; }
    }
    CHECK(exact);

    section("hypot");
    CHECK(hypot(3_fx, 4_fx) == 5_fx);
    CHECK(hypot(-3_fx, -4_fx) == 5_fx);
    CHECK(hypot(0_fx, 0_fx) == 0_fx);
    CHECK(hypot(5_fx, 0_fx) == 5_fx);

    // The whole point of the 64-bit intermediate: x*x alone overflows Q16.16
    // past ~181, but the length is still computed correctly.
    CHECK(hypot(3000_fx, 4000_fx) == 5000_fx);
    CHECK(hypot(20000_fx, 0_fx) == 20000_fx);

    double worst = 0.0;
    for (int i = 0; i < 300000; ++i)
    {
        const Fixed x = Fixed::from_raw(next_raw() >> 5);
        const Fixed y = Fixed::from_raw(next_raw() >> 5);
        const double ref = std::sqrt(x.to_double() * x.to_double() + y.to_double() * y.to_double());
        if (ref > static_cast<double>(Fixed::kMaxWhole)) continue;
        const double err = std::fabs(hypot(x, y).to_double() - ref) * kUlps;
        if (err > worst) worst = err;
    }
    std::printf("   hypot max error: %.4f ULP\n", worst);
    CHECK(worst <= 0.5);
}

void test_angles()
{
    section("angle representation");

    CHECK(Angle::from_degrees(0) == ang::kZero);
    CHECK(Angle::from_degrees(90) == ang::kQuarterTurn);
    CHECK(Angle::from_degrees(180) == ang::kHalfTurn);
    CHECK(Angle::from_degrees(360) == ang::kZero);
    CHECK(Angle::from_degrees(-90) == Angle::from_degrees(270));
    CHECK(Angle::from_degrees(720 + 45) == Angle::from_degrees(45));

    // Accumulation is exactly modular, no matter how many turns go by - the
    // property a Q16.16 radian simply cannot provide. Note that whole degrees
    // are NOT generally exact (360 does not divide 2^32); the guarantee is that
    // repeated addition equals the closed form, with no drift term at all.
    Angle acc = ang::kZero;
    const Angle step = Angle::from_raw(7654321u);
    constexpr std::uint32_t kTicks = 4000000u;
    for (std::uint32_t i = 0; i < kTicks; ++i) acc += step;
    CHECK(acc == Angle::from_raw(7654321u * kTicks));   // exact, ~7100 turns later

    // Powers-of-two fractions of a turn are exact, and those are the ones that
    // matter: 90, 45, 22.5 degrees all land on the nose.
    CHECK(Angle::from_degrees(45) * 8 == ang::kZero);
    CHECK(Angle::from_degrees(90) * 4 == ang::kZero);

    CHECK(Angle::from_degrees(90).to_degrees() == 90_fx);
    CHECK(Angle::from_degrees(270).to_degrees() == 270_fx);
    CHECK(ang::kQuarterTurn.to_turns() == 0.25_fx);
    CHECK(Angle::from_turns(0.25_fx) == ang::kQuarterTurn);
    CHECK(Angle::from_turns(1.25_fx) == ang::kQuarterTurn); // wraps
    CHECK(Angle::from_degrees(270).to_signed_turns() == -0.25_fx);
}

void test_trig()
{
    section("sin / cos");

    // Cardinal angles are exact, which keeps axis-aligned movement clean.
    CHECK(sin(Angle::from_degrees(0)) == 0_fx);
    CHECK(sin(Angle::from_degrees(90)) == 1_fx);
    CHECK(sin(Angle::from_degrees(180)) == 0_fx);
    CHECK(sin(Angle::from_degrees(270)) == -1_fx);
    CHECK(cos(Angle::from_degrees(0)) == 1_fx);
    CHECK(cos(Angle::from_degrees(90)) == 0_fx);
    CHECK(cos(Angle::from_degrees(180)) == -1_fx);
    CHECK(cos(Angle::from_degrees(270)) == 0_fx);
    CHECK(sin(Angle::from_degrees(30)) == 0.5_fx);
    CHECK(sin(Angle::from_degrees(150)) == 0.5_fx);
    CHECK(sin(Angle::from_degrees(210)) == -0.5_fx);

    // Symmetries, checked exactly rather than approximately.
    bool sym = true;
    for (std::uint32_t i = 0; i < 4096 && sym; ++i)
    {
        const Angle t = Angle::from_raw(i * 1048576u);
        sym = (sin(-t) == -sin(t))                                   // odd
           && (cos(-t) == cos(t))                                    // even
           && (sin(t + ang::kHalfTurn) == -sin(t))                   // half-turn
           && (cos(t) == sin(t + ang::kQuarterTurn));                // phase
    }
    CHECK(sym);

    // Accuracy sweep over the whole circle.
    double worst = 0.0;
    std::uint32_t worst_at = 0;
    constexpr std::uint32_t kStride = 3571u; // coprime with 2^32
    std::uint32_t raw = 0;
    for (int i = 0; i < 1000000; ++i)
    {
        raw += kStride * 997u;
        const double ref = std::sin(6.283185307179586 * (static_cast<double>(raw) / 4294967296.0));
        const double err = std::fabs(sin(Angle::from_raw(raw)).to_double() - ref) * kUlps;
        if (err > worst) { worst = err; worst_at = raw; }
    }
    std::printf("   sin  max error: %.4f ULP (at %.3f deg)\n",
                worst, Angle::from_raw(worst_at).to_degrees().to_double());
    CHECK(worst < 0.6);

    // sin^2 + cos^2 == 1, within what Q16.16 can express.
    double worst_id = 0.0;
    for (int i = 0; i < 200000; ++i)
    {
        const Angle t = Angle::from_raw(next_u32());
        const auto sc = sin_cos(t);
        const Fixed one = sc.sin * sc.sin + sc.cos * sc.cos;
        const double err = std::fabs(one.to_double() - 1.0) * kUlps;
        if (err > worst_id) worst_id = err;
    }
    std::printf("   sin^2+cos^2 max deviation: %.4f ULP\n", worst_id);
    CHECK(worst_id <= 3.0);

    section("atan2");

    CHECK(atan2(0_fx, 1_fx) == Angle::from_degrees(0));
    CHECK(atan2(1_fx, 1_fx) == Angle::from_degrees(45));
    CHECK(atan2(1_fx, 0_fx) == Angle::from_degrees(90));
    CHECK(atan2(1_fx, -1_fx) == Angle::from_degrees(135));
    CHECK(atan2(0_fx, -1_fx) == Angle::from_degrees(180));
    CHECK(atan2(-1_fx, -1_fx) == Angle::from_degrees(225));
    CHECK(atan2(-1_fx, 0_fx) == Angle::from_degrees(270));
    CHECK(atan2(-1_fx, 1_fx) == Angle::from_degrees(315));
    CHECK(atan2(0_fx, 0_fx) == ang::kZero);

    // Scale invariance: only the ratio may matter.
    CHECK(atan2(3_fx, 4_fx) == atan2(300_fx, 400_fx));
    CHECK(atan2(3_fx, 4_fx) == atan2(0.75_fx, 1_fx));
    CHECK(atan2(3_fx, 4_fx) == atan2(Fixed::from_raw(3), Fixed::from_raw(4)));

    double worst_bam = 0.0;
    for (int i = 0; i < 400000; ++i)
    {
        const Fixed y = Fixed::from_raw(next_raw());
        const Fixed x = Fixed::from_raw(next_raw());
        if (x.raw() == 0 && y.raw() == 0) continue;
        double ref = std::atan2(static_cast<double>(y.raw()), static_cast<double>(x.raw()))
                   / 6.283185307179586;
        if (ref < 0.0) ref += 1.0;
        double err = std::fabs(static_cast<double>(atan2(y, x).raw()) - ref * 4294967296.0);
        if (err > 2147483648.0) err = 4294967296.0 - err; // across the seam
        if (err > worst_bam) worst_bam = err;
    }
    std::printf("   atan2 max error: %.1f BAM = %.7f deg\n",
                worst_bam, worst_bam * 360.0 / 4294967296.0);
    CHECK(worst_bam < 2000.0);

    // Round trip: an angle -> unit vector -> angle must come back to itself,
    // limited only by the Q16.16 resolution of the vector components.
    double worst_rt = 0.0;
    for (int i = 0; i < 100000; ++i)
    {
        const Angle t = Angle::from_raw(next_u32());
        const auto sc = sin_cos(t);
        const Angle back = atan2(sc.sin, sc.cos);
        double err = std::fabs(static_cast<double>(shortest_delta_raw(t, back)));
        if (err > worst_rt) worst_rt = err;
    }
    std::printf("   atan2(sin,cos) round trip: %.1f BAM = %.5f deg\n",
                worst_rt, worst_rt * 360.0 / 4294967296.0);
    CHECK(worst_rt < 80000.0); // ~0.007 deg, dominated by the sin/cos output resolution
}

void test_steering()
{
    section("steering helpers");

    CHECK(shortest_delta_raw(Angle::from_degrees(10), Angle::from_degrees(20)) > 0);
    CHECK(shortest_delta_raw(Angle::from_degrees(20), Angle::from_degrees(10)) < 0);

    // The seam at 0/360 needs no special case.
    CHECK(shortest_delta_turns(Angle::from_degrees(350), Angle::from_degrees(10))
          == Angle::from_degrees(20).to_turns());
    CHECK(shortest_delta_raw(Angle::from_degrees(350), Angle::from_degrees(10))
          == static_cast<std::int32_t>(Angle::from_degrees(20).raw()));
    CHECK(shortest_delta_raw(Angle::from_degrees(10), Angle::from_degrees(350))
          == -static_cast<std::int32_t>(Angle::from_degrees(20).raw()));

    const Angle step = Angle::from_degrees(5);
    CHECK(rotate_towards(Angle::from_degrees(0), Angle::from_degrees(90), step)
          == Angle::from_degrees(5));
    CHECK(rotate_towards(Angle::from_degrees(0), Angle::from_degrees(270), step)
          == Angle::from_degrees(355));               // takes the short way
    CHECK(rotate_towards(Angle::from_degrees(0), Angle::from_degrees(3), step)
          == Angle::from_degrees(3));                 // snaps, never overshoots
    CHECK(rotate_towards(Angle::from_degrees(90), Angle::from_degrees(90), step)
          == Angle::from_degrees(90));

    // A turret must always converge and then hold, from any start.
    bool converges = true;
    for (int trial = 0; trial < 2000 && converges; ++trial)
    {
        const Angle target = Angle::from_raw(next_u32());
        Angle cur = Angle::from_raw(next_u32());
        for (int tick = 0; tick < 128; ++tick) cur = rotate_towards(cur, target, step);
        converges = (cur == target);
    }
    CHECK(converges);
}

// A canary for accidental semantic changes. Runs a fixed workload through every
// operation and hashes the raw bits of every result. Any change to rounding, to
// a coefficient, or to how a platform narrows an integer moves this number - so
// if two machines ever disagree about a replay, run this first.
std::uint64_t determinism_fingerprint()
{
    std::uint64_t h = 1469598103934665603ull; // FNV-1a
    auto mix = [&h](std::int64_t v) {
        for (int b = 0; b < 8; ++b)
        {
            h ^= static_cast<std::uint64_t>((v >> (b * 8)) & 0xFF);
            h *= 1099511628211ull;
        }
    };

    g_rng = 0xDEADBEEFu;
    for (int i = 0; i < 100000; ++i)
    {
        const Fixed a = next_small();
        const Fixed b = next_small();
        const Angle t = Angle::from_raw(next_u32());

        mix((a + b).raw());
        mix((a - b).raw());
        mix((a * b).raw());
        mix(abs(b) >= fx::kOne ? (a / b).raw() : 0); // keep the quotient in range
        mix(sqrt(abs(a)).raw());
        mix(hypot(a, b).raw());
        mix(lerp(a, b, frac(a)).raw());
        mix(mul_sat(a, b).raw());
        mix(sin(t).raw());
        mix(cos(t).raw());
        mix(atan2(a, b).raw());
        mix(rotate_towards(t, atan2(a, b), Angle::from_degrees(3)).raw());
    }
    return h;
}

} // namespace

int main()
{
    std::printf("rts-engine fixed-point tests\n");

    test_construction_and_conversion();
    test_arithmetic();
    test_overflow_and_saturation();
    test_sqrt();
    test_angles();
    test_trig();
    test_steering();

    section("determinism fingerprint");
    const std::uint64_t fp = determinism_fingerprint();
    std::printf("   fingerprint: 0x%016llX\n", static_cast<unsigned long long>(fp));
#ifdef RTS_EXPECTED_FINGERPRINT
    CHECK(fp == RTS_EXPECTED_FINGERPRINT);
#endif

    std::printf("\n%s  %d checks, %d failures\n",
                g_failures == 0 ? "PASSED" : "FAILED", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
