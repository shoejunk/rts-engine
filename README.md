RTS Engine
==========

2D line drawing
---------------

Run `run_debug.bat` (or `run_release.bat`) to build and open the drawing window.
The first configure downloads [SDL3 3.4.16](https://github.com/libsdl-org/SDL/releases/tag/release-3.4.16)
with a pinned SHA-256 hash. CMake 3.24+ and Visual Studio 2022 with the C++
workload are required by the Windows scripts. SDL is linked statically, so no
separate graphics DLL needs to be installed. Its license is in
`build/_deps/sdl3-src/LICENSE.txt` after configuring.

- **Draw:** press the left mouse button on the canvas to set the start, drag
  to preview the line, and release to set the end. Repeat to add more lines.
- **Save:** click **Save** or press **Ctrl+S**, choose a filename in the native
  Save dialog, and save all completed lines as CSV. The footer reports success,
  cancellation, or failure. Finish or cancel an active drag before saving.
- **Undo:** click **Undo** or press **Ctrl+Z** to remove the last line.
- **Cancel a drag:** press **Esc**. Switching away from the window also cancels
  an unfinished line.

Each saved row contains the two `(x,y)` endpoint pairs, in drawing order:

```csv
start_x,start_y,end_x,end_y
100,80,300,240
300,240,450,80
```

Coordinates are in canvas pixels: `(0,0)` is the top-left below the toolbar,
with x increasing rightward and y downward. There is no grid snapping; the
grid is a visual guide. Releasing outside the canvas preserves the actual
endpoint (which may be negative). Resizing does not rescale stored lines.
CSV preserves fractional coordinates and excludes the in-progress preview.
Saving again replaces the selected file with the complete current drawing.
Drawings stay in memory until the window closes; save before closing.

The `rts_line_drawing_tests` CTest target covers endpoint capture, cancellation,
undo, exact CSV output, non-ASCII filenames, decimal locales, and save failures.

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

Coarse collision triangulation
------------------------------

`src/geometry/cdt.h` provides the header-only `rts::Cdt<GridFractionBits>`;
link the CMake target `rts_geometry`. It incrementally triangulates a bounded
rectangle, inserts points and constrained segments, and exposes CCW triangles
with twin-edge adjacency and constraint flags. It follows the coarse-coordinate
idea in [James Anhalt's CDT](https://github.com/jeaiii/ce/blob/main/lib/h/ce/cdt.h),
with an independent implementation using our `Fixed` API and portable exact
integer predicates.

Collision precision is deliberately separate from simulation precision:

| Type | Grid step | Fixed raw units per step |
|------|-----------|--------------------------|
| `Fixed` | 1/16384 unit | 1 |
| `Cdt<>` / `Cdt<4>` (default) | 1/16 unit | 1024 |
| `Cdt<2>` | 1/4 unit | 4096 |
| `Cdt<0>` | 1 unit | 16384 |

Choose the grid once for a map; it is not a per-point tolerance. Input points
and map bounds snap to the nearest grid coordinate, ties away from zero.
Vertices snapping to the same position share one vertex ID. This limits detail
to the chosen grid without reducing the precision of unit movement. It does
not simplify arbitrary collinear chains or guarantee a minimum passage width.
Snapping can move a wall by up to half a grid step on each axis; author clearance
against the snapped geometry.

```cpp
#include "geometry/cdt.h"
using namespace rts;
using namespace rts::literals;

Cdt<> collision; // 1/16-unit collision grid
auto error = collision.reset({-1024_fx, -1024_fx}, {1024_fx, 1024_fx});
if (error != Cdt<>::Error::none) return;

// Horizontal maps supply world x/z as the point's x/y coordinates.
const std::array<CdtPoint, 4> building{{
    {10_fx, 10_fx}, {18_fx, 10_fx}, {18_fx, 16_fx}, {10_fx, 16_fx}
}};
error = collision.insert_polyline(building, true);
if (error != Cdt<>::Error::none) return;

const auto face_id = collision.locate({4.125_fx, 5.25_fx});
if (face_id != Cdt<>::kInvalid)
{
    const auto& face = collision.triangles()[face_id];
    const CdtPoint a = collision.position(face.vertices[0]);
    const bool wall = face.constrained(0);
    const auto twin = face.twins[0];
    // If twin != kInvalid: adjacent face = twin / 3, edge = twin % 3.
    // Edge i runs vertices[i] -> vertices[(i + 1) % 3].
}
```

`insert_point` returns an error, vertex ID and whether a new vertex was added.
`insert_constraint(a, b)` accepts existing vertex IDs; `insert_edge(a, b)` accepts
`CdtPoint` endpoints. `insert_polyline(points, closed)` batches segments in one
transaction, discarding consecutive snapped duplicates and a repeated closing
vertex. An entirely collapsed segment or loop reports `collapsed_constraint`.

Existing vertices on a segment, T-junctions, and intersections on the chosen
grid split constraints automatically. Overlapping constraints are idempotent;
later point insertions preserve both halves of a split wall. A crossing between
grid positions returns `off_grid_intersection`: resolve it in map authoring or
choose a different grid. Constraint/polyline failures leave the whole mesh
unchanged, including endpoints tentatively inserted by that operation.

Bounds and coordinates that cannot be represented after snapping are rejected
in both Debug and Release. Grid coordinates use `int32_t`, retaining the world
range of `Fixed` (apart from its upper fractional tail when rounding would
overflow). Orientation uses 64-bit intermediates; in-circle tests use a small
portable 128-bit integer accumulator. Cocircular edges stay unchanged. Identical
ordered inputs produce identical topology; different insertion orders can choose
different valid diagonals in cocircular configurations. `GridFractionBits` may
range from zero through `Fixed::kFracBits - 2`.

The four rectangle corners are vertices 0-3, starting at lower left and going
CCW; boundary edges are constrained. `vertices()` exposes the integer grid;
use `position(id)` for `Fixed` world coordinates. Vertex IDs survive insertions.
Reacquire triangle/edge contents and spans after successful mutations; reset
invalidates all IDs. `locate` uses the full query precision, returns the first
incident triangle for an edge/vertex hit, and returns `kInvalid` outside the map.

This is map geometry infrastructure. Closed loops do not mark or remove their
interiors, and there is no obstacle removal, radius clearance, collision sweep,
or pathfinding policy yet. Point location and edge lookup currently scan the
mesh; constraint transactions copy it. Batch map boundaries with `insert_polyline`
and profile representative map sizes before using edits in a per-tick workload.

`rts_cdt_tests` checks snapping, range failures and rollback, boundary and wall
splits, collinear overlaps, nested/concave loops, skinny cells, randomized
constraint insertion, and large-coordinate predicates. It independently checks
positive areas, rectangle coverage, reciprocal twins, planar edges, and local
Delaunay legality on small maps, plus topology agreement after large scaling
and translation. A fixed workload pins the mesh fingerprint to
`0x8F184EBD8BEA3EDB`, verified in MSVC Debug and Release. The same CTest commands
above run both math and CDT suites.
