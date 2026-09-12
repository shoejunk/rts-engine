RTS Engine
==========

Deterministic math layer
------------------------

A lockstep RTS only sends inputs across the network, so every client must
compute bit-identical results from them. Floating point cannot promise that
across machines, compilers and optimisation levels, so the simulation runs
entirely on fixed-point types.

| Header              | Provides                                            |
|---------------------|-----------------------------------------------------|
| `src/math/fixed.h`  | `Fixed` - 1.17.14 scalar, arithmetic, `sqrt`, `hypot` |
| `src/math/angle.h`  | `Angle` - binary angles, `sin`/`cos`/`atan2`, steering |

Both are header-only and fully `constexpr`; the CMake target is `rts_math`.

### `Fixed` - 1.17.14

`int32_t` storage: **1 sign bit, 17 integer bits, 14 fraction bits**. The same
layout is also written "signed 18.14" - that convention just folds the sign bit
into the whole part. Encoding is two's complement, which C++20 mandates anyway.

```
range      [-131072, +131071.99993896484375]
resolution 1/16384 = 0.00006103515625
```

The range is asymmetric, as two's complement always is: `-kMin` overflows and
`abs(kMin)` comes back negative. That is inherent - the header asserts on it
rather than papering over it.

Every product and quotient is formed in an `int64_t` intermediate, so the
operation itself is exact and only the store back to 32 bits rounds.

`Fixed::kFracBits` is the single knob. Every constant, shift and conversion in
both headers derives from it, so changing the split is a one-line edit - the
test suite is written against the format rather than against literal raw values,
and passes unchanged at other splits (only the two assertions that deliberately
pin the 1.17.14 range, and the determinism fingerprint, need re-baselining).

```cpp
#include "math/fixed.h"
using namespace rts;
using namespace rts::literals;

Fixed speed    = 4.5_fx;            // consteval literal
Fixed dt       = Fixed::from_raw(546);   // 1/30 s as a raw value
Fixed distance = speed * dt;
Fixed range    = hypot(dx, dy);     // 64-bit intermediate: no overflow at scale
```

Key choices:

- **Explicit construction.** `Fixed(3)` converts an integer; there is no
  implicit conversion, so int/fixed mixing never happens silently.
- **`from_num` is `consteval`.** Real literals can only be converted at compile
  time, which makes it impossible for a runtime `float` to leak into the sim.
  `to_double()` is one-way, for rendering, UI and logging.
- **Zero default.** `Fixed x;` is `0`, not uninitialised memory.
- **Overflow wraps** (modular, defined, reproducible) rather than trapping.
  `add_sat` / `sub_sat` / `mul_sat` clamp instead, where that is what you want.
  Debug builds assert on overflow (`RTS_FIXED_CHECKS`).
- **Round-to-nearest, ties away from zero**, everywhere. Truncation carries a
  -0.5 ULP bias that accumulates into visible drift over a long match. Ties away
  from zero (rather than the cheaper toward-`+inf`) is what makes the type
  symmetric under negation, so a mirrored army evolves as the exact mirror of
  the original.
- **`mul_div(a, b, c)`** computes `a*b/c` with one 64-bit intermediate and a
  single rounding step, for when the product alone would not fit.

### `Angle` - binary angle measurement

A `uint32_t` where a full turn is exactly `2^32`. Wrapping is unsigned overflow,
so it is free and, more importantly, *exact*: a turn rate added every tick for
an hour never drifts. The quadrant is the top two bits, so argument reduction is
a shift and a mask. Resolution is `2e-8` degrees in the same four bytes a
fixed-point radian would occupy.

```cpp
#include "math/angle.h"

Angle facing = Angle::from_degrees(45);
auto  sc     = sin_cos(facing);              // both at once
Angle want   = atan2(target_y - y, target_x - x);
facing       = rotate_towards(facing, want, turn_rate);   // takes the short way
```

`sin`/`cos`/`atan2` use odd minimax polynomials evaluated by Horner in Q2.30
with `int64` products - no tables (8 KB of sine table costs more in cache misses
across thousands of units per tick than the four multiplies it saves) and no
CORDIC (~30 dependent iterations to save those same multiplies).

Whole degrees are *not* exactly representable, since 360 is not a power of two -
only multiples of 45 are. Accumulate in raw BAM, not in degrees.

### Measured accuracy

Produced by the test suite, not estimated:

| Operation        | Max error                               |
|------------------|-----------------------------------------|
| `*`, `/`, `hypot`| 0.5 ULP (the quantisation floor)        |
| `sqrt`           | 0.5 ULP, correctly rounded; exact on perfect squares |
| `sin`, `cos`     | 0.516 ULP; exact at 0/90/180/270 degrees |
| `atan2`          | 1839 BAM = 0.00015 degrees; exact at all eight cardinals |

One ULP is `1/16384`. `atan2` is independent of the scalar format - it consumes
a ratio and produces a BAM angle - which is why its figure is unchanged from
other splits.

### Tests

```bash
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The suite checks exact integer properties wherever it can (sqrt rounding,
commutativity, negation symmetry, modular wrapping) rather than settling for
float comparisons, and measures error against `std::` references everywhere
else. It ends with a **determinism fingerprint**: an FNV-1a hash over a fixed
workload run through every operation. MSVC Debug and Release both produce
`0xC4A58C732BE3F827`, and CMake asserts that value. If a change or a new
platform moves it, replays recorded before the change will desync - so add
every toolchain you ship on to CI and compare this number first. It is
baselined for 1.17.14; changing `kFracBits` changes it by design.

### What this does not do

The math layer alone does not make a simulation deterministic. Still to watch:
iteration order of hash containers, pointer-value comparisons, unstable sorts,
uninitialised memory, and any `float` in simulation state.
