#include "geometry/cdt.h"

#include <cstdio>
#include <set>

using namespace rts;
using namespace rts::literals;

namespace {
int checks = 0, failures = 0;
#define CHECK(...) do { ++checks; if (!(__VA_ARGS__)) { ++failures; \
    std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (false)
using Mesh = Cdt<>;
using Error = Mesh::Error;
using Id = Mesh::VertexId;
using Point = Mesh::GridPoint;

CdtPoint p(int x, int y) { return {Fixed(x), Fixed(y)}; }
CdtPoint lattice(int x, int y)
{
    return {Fixed::from_raw(x * Mesh::kGridStepRaw), Fixed::from_raw(y * Mesh::kGridStepRaw)};
}
std::int64_t area(Point a, Point b, Point c)
{
    return (std::int64_t{b.x} - a.x) * (std::int64_t{c.y} - a.y)
         - (std::int64_t{c.x} - a.x) * (std::int64_t{b.y} - a.y);
}
bool on(Point a, Point b, Point q)
{
    return area(a, b, q) == 0 && q.x >= std::min(a.x, b.x) && q.x <= std::max(a.x, b.x)
                             && q.y >= std::min(a.y, b.y) && q.y <= std::max(a.y, b.y);
}
bool opposite(std::int64_t a, std::int64_t b) { return (a < 0 && b > 0) || (a > 0 && b < 0); }

// Independent, ordinary int64 determinant for small test maps. Its unshifted
// formula differs from the production predicate, and no production geometry
// helper is used for mesh validation.
std::int64_t circle(Point a, Point b, Point c, Point d)
{
    const auto norm = [](Point q) { return std::int64_t{q.x} * q.x + std::int64_t{q.y} * q.y; };
    return norm(a) * area(b, c, d) - norm(b) * area(a, c, d)
         + norm(c) * area(a, b, d) - norm(d) * area(a, b, c);
}

bool has_edge(const Mesh& mesh, Id a, Id b, bool wall = true)
{
    for (const auto& f : mesh.triangles())
        for (unsigned i = 0; i < 3; ++i)
            if (((f.vertices[i] == a && f.vertices[(i + 1) % 3] == b)
                || (f.vertices[i] == b && f.vertices[(i + 1) % 3] == a))
                && (!wall || f.constrained(i))) return true;
    return false;
}

bool has_wall(const Mesh& mesh, CdtPoint a, CdtPoint b)
{
    const auto sa = Mesh::snap(a), sb = Mesh::snap(b);
    if (!sa || !sb) return false;
    const Point pa{sa->x.raw() / Mesh::kGridStepRaw, sa->y.raw() / Mesh::kGridStepRaw};
    const Point pb{sb->x.raw() / Mesh::kGridStepRaw, sb->y.raw() / Mesh::kGridStepRaw};
    std::vector<Id> ids;
    for (Id v = 0; v < mesh.vertices().size(); ++v)
        if (on(pa, pb, mesh.vertices()[v])) ids.push_back(v);
    std::sort(ids.begin(), ids.end(), [&](Id u, Id v) {
        const auto x = mesh.vertices()[u], y = mesh.vertices()[v];
        return x.x == y.x ? x.y < y.y : x.x < y.x;
    });
    if (ids.size() < 2) return false;
    for (std::size_t i = 1; i < ids.size(); ++i)
        if (!has_edge(mesh, ids[i - 1], ids[i])) return false;
    return true;
}

// Topology, geometric embedding, exact area coverage, all vertices used, and
// local constrained-Delaunay legality. The O(E^2) crossing checks are test-only.
void validate(const Mesh& mesh, bool small = true)
{
    const auto vs = mesh.vertices();
    const auto fs = mesh.triangles();
    std::int64_t total_area = 0;
    std::set<std::pair<Id, Id>> edges;
    std::set<Id> used;
    for (std::size_t n = 0; n < fs.size(); ++n)
    {
        const auto& f = fs[n];
        CHECK(f.vertices[0] < vs.size() && f.vertices[1] < vs.size() && f.vertices[2] < vs.size());
        const auto a = vs[f.vertices[0]], b = vs[f.vertices[1]], c = vs[f.vertices[2]];
        const auto ar = area(a, b, c);
        CHECK(ar > 0);
        total_area += ar;
        for (unsigned i = 0; i < 3; ++i)
        {
            const auto u = f.vertices[i], v = f.vertices[(i + 1) % 3];
            used.insert(u);
            edges.emplace(std::min(u, v), std::max(u, v));
            const auto t = f.twins[i];
            if (t == Mesh::kInvalid) { CHECK(f.constrained(i)); continue; }
            CHECK(t / 3 < fs.size());
            const auto& other = fs[t / 3];
            CHECK(other.twins[t % 3] == n * 3 + i);
            CHECK(other.vertices[t % 3] == v && other.vertices[(t % 3 + 1) % 3] == u);
            CHECK(other.constrained(t % 3) == f.constrained(i));
            if (small && !f.constrained(i)) CHECK(circle(a, b, c, vs[other.vertices[(t % 3 + 2) % 3]]) <= 0);
        }
    }
    CHECK(total_area == 2 * area(vs[0], vs[1], vs[2]));
    CHECK(used.size() == vs.size());
    CHECK(vs.size() + fs.size() == edges.size() + 1);
    for (const auto& [a, b] : edges)
    {
        for (Id v = 0; v < vs.size(); ++v)
            if (v != a && v != b) CHECK(!on(vs[a], vs[b], vs[v]));
        for (const auto& [c, d] : edges)
            if (a != c && a != d && b != c && b != d)
                CHECK(!(opposite(area(vs[a], vs[b], vs[c]), area(vs[a], vs[b], vs[d]))
                     && opposite(area(vs[c], vs[d], vs[a]), area(vs[c], vs[d], vs[b]))));
    }
}

bool same(const Mesh& a, const Mesh& b)
{
    return std::ranges::equal(a.vertices(), b.vertices()) && std::ranges::equal(a.triangles(), b.triangles());
}

void wide_arithmetic()
{
    // Expected limbs computed independently with .NET BigInteger. These cover
    // cross-limb carries, negative products, and the signed 64-bit extremes.
    struct Case { std::int64_t a, b; std::uint64_t lo, hi; };
    constexpr Case cases[] = {
        {2305843009213693951ll, 2305843009213693907ll, 0x400000000000002Dull, 0x03FFFFFFFFFFFFFAull},
        {-2305843009213693951ll, 2305843009213693907ll, 0xBFFFFFFFFFFFFFD3ull, 0xFC00000000000005ull},
        {INT64_MAX, INT64_MAX, 1, 0x3FFFFFFFFFFFFFFFull},
        {INT64_MIN, INT64_MIN, 0, 0x4000000000000000ull},
        {INT64_MIN, 1, 0x8000000000000000ull, UINT64_MAX},
        {4294967295ll, 4294967295ll, 0xFFFFFFFE00000001ull, 0},
        {4294967297ll, 4294967297ll, 0x0000000200000001ull, 1},
        {-4294967297ll, 4294967297ll, 0xFFFFFFFDFFFFFFFFull, 0xFFFFFFFFFFFFFFFEull}
    };
    for (const auto& c : cases)
    {
        const auto value = cdt_detail::Wide::product(c.a, c.b);
        CHECK(value.lo == c.lo && value.hi == c.hi);
        CHECK(value.sign() == ((c.a < 0) != (c.b < 0) ? -1 : 1));
    }
    auto sum = cdt_detail::Wide::product(cases[0].a, cases[0].b);
    sum += cdt_detail::Wide::product(-1, 1);
    sum += cdt_detail::Wide::product(cases[1].a, cases[1].b);
    CHECK(sum.lo == UINT64_MAX && sum.hi == UINT64_MAX && sum.sign() == -1);
    sum += cdt_detail::Wide::product(1, 1);
    CHECK(sum.sign() == 0);
    sum += cdt_detail::Wide::product(1, 1);
    CHECK(sum.lo == 1 && sum.hi == 0 && sum.sign() == 1);
}

void snapping_and_errors()
{
    static_assert(Mesh::kGridStep == 0.0625_fx);
    static_assert(Mesh::snap({0.03125_fx, -0.03125_fx}) == CdtPoint{0.0625_fx, -0.0625_fx});
    static_assert(Cdt<2>::kGridStep == 0.25_fx);
    static_assert(Cdt<0>::kGridStep == 1_fx);
    CHECK(!Mesh::snap({fx::kMax, 0_fx}));
    CHECK(Mesh::snap({fx::kMin, 0_fx})->x == fx::kMin);
    Mesh mesh;
    CHECK(mesh.insert_point(p(0, 0)).error == Error::not_initialized);
    CHECK(mesh.insert_constraint(0, 1) == Error::not_initialized);
    CHECK(mesh.locate(p(0, 0)) == Mesh::kInvalid);
    CHECK(mesh.reset(p(-10, -10), p(10, 10)) == Error::none);
    validate(mesh);
    const auto a = mesh.insert_point({1_fx, 2_fx});
    const auto b = mesh.insert_point({Fixed::from_raw(Fixed::kRawOne + 1), 2_fx});
    CHECK(a && b && a.inserted && !b.inserted && a.vertex == b.vertex);
    CHECK(mesh.position(a.vertex) == CdtPoint{1_fx, 2_fx});
    const auto before = mesh;
    CHECK(mesh.insert_point(p(20, 0)).error == Error::outside_bounds);
    CHECK(mesh.insert_point({fx::kMax, 0_fx}).error == Error::coordinate_out_of_range);
    CHECK(mesh.insert_constraint(Mesh::kInvalid, 0) == Error::invalid_vertex);
    CHECK(mesh.insert_constraint(a.vertex, a.vertex) == Error::collapsed_constraint);
    CHECK(mesh.insert_edge(p(0, 0), {Fixed::from_raw(1), 0_fx}) == Error::collapsed_constraint);
    CHECK(mesh.insert_edge(p(3, 4), p(30, 0)) == Error::outside_bounds);
    CHECK(mesh.reset(p(1, 1), p(1, 3)) == Error::invalid_bounds);
    CHECK(mesh.reset(p(0, 0), {fx::kMax, 1_fx}) == Error::coordinate_out_of_range);
    CHECK(same(mesh, before));
    CHECK(mesh.locate({Fixed::from_raw(10 * Fixed::kRawOne + 1), 0_fx}) == Mesh::kInvalid);
    CHECK(mesh.locate(p(10, 0)) != Mesh::kInvalid);
    CHECK(mesh.locate({Fixed::from_raw(10 * Fixed::kRawOne - 1), 0_fx}) != Mesh::kInvalid);

    Cdt<0> coarse;
    CHECK(coarse.reset(p(-10, -10), p(10, 10)) == Cdt<0>::Error::none);
    const auto c = coarse.insert_point({1.125_fx, -2.25_fx});
    CHECK(c && coarse.position(c.vertex) == p(1, -2));
}

void point_insertion()
{
    Mesh mesh;
    CHECK(mesh.reset(p(-10, -10), p(10, 10)) == Error::none);
    // Shared diagonal, all four boundary sides, existing corners, and faces.
    for (const auto q : {p(0, 0), p(-10, 0), p(0, -10), p(10, 0), p(0, 10), p(-10, -10),
                         p(-4, 2), p(3, -2), p(3, 6), p(-2, -7)})
    {
        CHECK(mesh.insert_point(q));
        CHECK(mesh.locate(q) != Mesh::kInvalid);
        validate(mesh);
    }
    CHECK(has_wall(mesh, p(-10, -10), p(-10, 10)));
    CHECK(has_wall(mesh, p(-10, 10), p(10, 10)));
    CHECK(has_wall(mesh, p(10, 10), p(10, -10)));
    CHECK(has_wall(mesh, p(10, -10), p(-10, -10)));
}

void constraints()
{
    Mesh mesh;
    CHECK(mesh.reset(p(-20, -20), p(20, 20)) == Error::none);
    for (const auto q : {p(-7, -4), p(3, 4), p(8, -6), p(-9, 9), p(2, -8), p(0, 0)})
        CHECK(mesh.insert_point(q));
    CHECK(mesh.insert_edge(p(-15, 0), p(15, 0)) == Error::none);
    CHECK(mesh.insert_edge(p(0, -15), p(0, 15)) == Error::none);
    CHECK(mesh.insert_edge(p(-10, -10), p(10, 10)) == Error::none);
    validate(mesh);
    CHECK(has_wall(mesh, p(-15, 0), p(15, 0)));
    CHECK(has_wall(mesh, p(0, -15), p(0, 15)));
    CHECK(has_wall(mesh, p(-10, -10), p(10, 10)));
    CHECK(mesh.insert_edge(p(12, 0), p(-12, 0)) == Error::none); // reversed partial overlap
    CHECK(mesh.insert_point(p(5, 0))); // constrained edge split
    CHECK(mesh.insert_point(p(5, 5)));
    validate(mesh);
    CHECK(has_wall(mesh, p(-15, 0), p(15, 0)));
    CHECK(has_wall(mesh, p(-10, -10), p(10, 10)));

    Mesh crossing;
    CHECK(crossing.reset(p(-10, -10), p(10, 10)) == Error::none);
    CHECK(crossing.insert_edge(p(-8, -3), p(8, 5)) == Error::none);
    const auto count = crossing.vertices().size();
    CHECK(crossing.insert_edge(p(0, -8), p(0, 8)) == Error::none);
    CHECK(crossing.vertices().size() == count + 3); // two endpoints + new (0,1)
    CHECK(has_wall(crossing, p(-8, -3), p(8, 5)));
    CHECK(has_wall(crossing, p(0, -8), p(0, 8)));
    validate(crossing);

    Mesh off_grid;
    CHECK(off_grid.reset(p(-10, -10), p(10, 10)) == Error::none);
    CHECK(off_grid.insert_edge(lattice(-3, 0), lattice(3, 1)) == Error::none);
    const auto saved = off_grid;
    CHECK(off_grid.insert_edge(lattice(0, -3), lattice(0, 3)) == Error::off_grid_intersection);
    CHECK(same(off_grid, saved));
    const std::array late_failure{lattice(-5, -5), lattice(5, -5), lattice(0, -3), lattice(0, 3)};
    CHECK(off_grid.insert_polyline(late_failure) == Error::off_grid_intersection);
    CHECK(same(off_grid, saved)); // rollback earlier successful segments too
    validate(off_grid);
}

void polylines_and_degeneracies()
{
    Mesh mesh;
    CHECK(mesh.reset(p(-20, -20), p(20, 20)) == Error::none);
    const std::array loop{p(-8, -8), p(8, -8), p(8, 8), p(2, 8), p(2, 0), p(-8, 0)};
    CHECK(mesh.insert_polyline(loop, true) == Error::none);
    for (std::size_t i = 0; i < loop.size(); ++i) CHECK(has_wall(mesh, loop[i], loop[(i + 1) % loop.size()]));
    // Nested obstacle loops and repeated closing/consecutive points.
    const std::array inner{p(-4, -6), p(-4, -6), p(-1, -6), p(-1, -2), p(-4, -2), p(-4, -6)};
    CHECK(mesh.insert_polyline(inner, true) == Error::none);
    CHECK(mesh.locate(p(-2, -4)) != Mesh::kInvalid); // a loop does not remove faces
    validate(mesh);
    for (int x = -18; x <= 18; x += 3) CHECK(mesh.insert_point(p(x, 12)));
    CHECK(mesh.insert_edge(p(-18, 12), p(18, 12)) == Error::none);
    CHECK(has_wall(mesh, p(-18, 12), p(18, 12)));
    validate(mesh);

    Mesh thin;
    CHECK(thin.reset(lattice(-100, 0), lattice(100, 1)) == Error::none);
    for (int x = -90; x <= 90; x += 10)
    {
        CHECK(thin.insert_point(lattice(x, 0)));
        CHECK(thin.insert_point(lattice(x, 1)));
    }
    CHECK(thin.insert_edge(lattice(-90, 0), lattice(90, 1)) == Error::none);
    validate(thin);
}

std::uint32_t rng = 0x56789ABCu;
std::uint32_t random_u32() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

void randomized()
{
    for (int trial = 0; trial < 18; ++trial)
    {
        Mesh mesh, replay;
        CHECK(mesh.reset(lattice(-120, -120), lattice(120, 120)) == Error::none);
        CHECK(replay.reset(lattice(-120, -120), lattice(120, 120)) == Error::none);
        std::vector<CdtPoint> points;
        for (int i = 0; i < 45; ++i)
        {
            const auto x = static_cast<int>(random_u32() % 221) - 110;
            const auto y = static_cast<int>(random_u32() % 221) - 110;
            points.push_back(lattice(x, y));
            CHECK(mesh.insert_point(points.back()));
            CHECK(replay.insert_point(points.back()));
        }
        validate(mesh);
        std::vector<std::pair<CdtPoint, CdtPoint>> walls;
        // A fan cannot have off-grid crossings; it exercises long constraint
        // corridors through arbitrary triangulations in many directions.
        for (std::size_t i = 1; i < 16; ++i)
        {
            const auto error = mesh.insert_edge(points[0], points[i]);
            if (error != Error::none)
                std::printf("fan trial %d edge %zu error %d: (%d,%d)-(%d,%d)\n", trial, i, static_cast<int>(error),
                    points[0].x.raw() / Mesh::kGridStepRaw, points[0].y.raw() / Mesh::kGridStepRaw,
                    points[i].x.raw() / Mesh::kGridStepRaw, points[i].y.raw() / Mesh::kGridStepRaw);
            CHECK(error == Error::none);
            CHECK(replay.insert_edge(points[0], points[i]) == error);
            walls.emplace_back(points[0], points[i]);
        }
        validate(mesh);
        for (const auto& [a, b] : walls) CHECK(has_wall(mesh, a, b));
        CHECK(same(mesh, replay));
        for (int i = 0; i < 12; ++i)
        {
            const auto a = points[random_u32() % points.size()], b = points[random_u32() % points.size()];
            const auto saved = mesh;
            const auto error = mesh.insert_edge(a, b);
            CHECK(error == Error::none || error == Error::off_grid_intersection || error == Error::collapsed_constraint);
            if (error == Error::none) walls.emplace_back(a, b);
            else CHECK(same(mesh, saved));
        }
        validate(mesh);
        for (const auto& [a, b] : walls) CHECK(has_wall(mesh, a, b));
    }
}

void large_coordinates()
{
    // The in-circle terms on this map exceed 64 bits by a large margin.
    Mesh mesh;
    CHECK(mesh.reset(p(-131072, -131072), p(131071, 131071)) == Error::none);
    for (const auto q : {p(0, 0), p(100000, 0), p(0, 100000), p(-100000, 0), p(0, -100000),
                         p(1, 1), p(-1, -1), p(80000, 80001), p(-80000, 80000)}) CHECK(mesh.insert_point(q));
    CHECK(mesh.insert_edge(p(-120000, 20), p(120000, 20)) == Error::none);
    CHECK(mesh.insert_edge(p(30, -120000), p(30, 120000)) == Error::none);
    CHECK(has_wall(mesh, p(-120000, 20), p(120000, 20)));
    CHECK(has_wall(mesh, p(30, -120000), p(30, 120000)));
    validate(mesh, false);
    CHECK(mesh.locate({fx::kMax, fx::kMax}) == Mesh::kInvalid);
    CHECK(mesh.locate({fx::kMin, fx::kMin}) != Mesh::kInvalid);

    // Translation and power-of-two scaling cannot change exact Delaunay
    // decisions. Exercise cancellation at large world offsets as well.
    Mesh small, big, translated;
    CHECK(small.reset(p(-16, -16), p(16, 16)) == Error::none);
    CHECK(big.reset(p(-65536, -65536), p(65536, 65536)) == Error::none);
    CHECK(translated.reset(p(99984, 99984), p(100016, 100016)) == Error::none);
    for (const auto xy : {std::pair{0, 0}, {7, 3}, {-7, 3}, {-3, -8}, {9, -5}, {8, 11}, {-8, -9}})
    {
        CHECK(small.insert_point(p(xy.first, xy.second)));
        CHECK(big.insert_point(p(xy.first * 4096, xy.second * 4096)));
        CHECK(translated.insert_point(p(100000 + xy.first, 100000 + xy.second)));
    }
    CHECK(std::ranges::equal(small.triangles(), big.triangles()));
    CHECK(std::ranges::equal(small.triangles(), translated.triangles()));
    validate(small);
    validate(big, false);
    validate(translated, false);

    // Widest supported lattice: four Fixed raw units per cell.
    Cdt<Fixed::kFracBits - 2> fine;
    using FineError = decltype(fine)::Error;
    CHECK(fine.reset({fx::kMin, fx::kMin}, {Fixed::from_raw(INT32_MAX - 3), Fixed::from_raw(INT32_MAX - 3)}) == FineError::none);
    CHECK(fine.insert_point(p(0, 0)));
    CHECK(fine.insert_point(p(100000, 100000)));
    CHECK(fine.insert_point({Fixed::from_raw(4), Fixed::from_raw(-4)}));
    CHECK(fine.insert_edge(p(-120000, 0), p(120000, 0)) == FineError::none);
    CHECK(fine.insert_edge(p(0, -120000), p(0, 120000)) == FineError::none);
}

std::uint64_t fingerprint()
{
    Mesh mesh;
    CHECK(mesh.reset(p(-64, -64), p(64, 64)) == Error::none);
    rng = 0xBADF00Du;
    for (int i = 0; i < 120; ++i)
    {
        const auto x = static_cast<int>(random_u32() % 1801) - 900;
        const auto y = static_cast<int>(random_u32() % 1801) - 900;
        CHECK(mesh.insert_point(lattice(x, y)));
    }
    for (int i = -48; i <= 48; i += 12)
    {
        CHECK(mesh.insert_edge(p(-60, i), p(60, i)) == Error::none);
        CHECK(mesh.insert_edge(p(i, -60), p(i, 60)) == Error::none);
    }
    for (int i = -48; i <= 48; i += 12)
    {
        CHECK(has_wall(mesh, p(-60, i), p(60, i)));
        CHECK(has_wall(mesh, p(i, -60), p(i, 60)));
    }
    validate(mesh);
    std::uint64_t hash = 14695981039346656037ull;
    const auto mix = [&](std::uint32_t value) {
        for (unsigned i = 0; i < 4; ++i)
        {
            hash ^= (value >> (i * 8)) & 255u;
            hash *= 1099511628211ull;
        }
    };
    for (const auto v : mesh.vertices()) { mix(static_cast<std::uint32_t>(v.x)); mix(static_cast<std::uint32_t>(v.y)); }
    for (const auto& f : mesh.triangles())
    {
        for (const auto v : f.vertices) mix(v);
        for (const auto t : f.twins) mix(t);
        mix(f.constraint_mask);
    }
    return hash;
}

} // namespace

int main()
{
    wide_arithmetic();
    snapping_and_errors();
    point_insertion();
    constraints();
    polylines_and_degeneracies();
    randomized();
    large_coordinates();
    const auto hash = fingerprint();
    std::printf("CDT fingerprint: 0x%016llX\n", static_cast<unsigned long long>(hash));
    // Golden topology for this ordered workload, verified with MSVC Debug and
    // Release. Include every shipping toolchain in CI before promising lockstep.
    CHECK(hash == 0x8F184EBD8BEA3EDBull);
    std::printf("CDT: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
