#pragma once
// Deterministic constrained Delaunay triangulation of a rectangular map.
// Inspired by the coarse-coordinate CDT interface at
// https://github.com/jeaiii/ce/blob/main/lib/h/ce/cdt.h; independent implementation.
//
// Fixed is the world-coordinate API. Stored vertices use a separate integer
// lattice: by default 1/16 world unit, versus Fixed's 1/16384. All topology
// predicates are exact integer operations, including the in-circle predicate.
// There are no floating-point tolerances or off-lattice Steiner vertices.

#include "math/fixed.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <deque>
#include <map>
#include <numeric>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace rts {

struct CdtPoint
{
    Fixed x, y; // Use world x/z here for a horizontal 3D map.
    bool operator==(const CdtPoint&) const = default;
};

namespace cdt_detail {

// Only multiplication, addition and sign are needed from 128-bit arithmetic.
// Unsigned limbs keep this portable to MSVC as well as Clang/GCC, without
// compiler extensions, signed overflow, or a multiprecision dependency.
struct Wide
{
    std::uint64_t lo = 0, hi = 0;

    static constexpr Wide product(std::int64_t a, std::int64_t b) noexcept
    {
        const auto u = a < 0 ? std::uint64_t{0} - static_cast<std::uint64_t>(a)
                             : static_cast<std::uint64_t>(a);
        const auto v = b < 0 ? std::uint64_t{0} - static_cast<std::uint64_t>(b)
                             : static_cast<std::uint64_t>(b);
        constexpr std::uint64_t mask = 0xFFFFFFFFull;
        const auto p00 = (u & mask) * (v & mask);
        const auto p01 = (u & mask) * (v >> 32);
        const auto p10 = (u >> 32) * (v & mask);
        const auto p11 = (u >> 32) * (v >> 32);
        const auto mid = (p00 >> 32) + (p01 & mask) + (p10 & mask);
        Wide w{(p00 & mask) | (mid << 32),
               p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32)};
        if ((a < 0) != (b < 0))
        {
            w.lo = ~w.lo + 1;
            w.hi = ~w.hi + (w.lo == 0 ? 1u : 0u);
        }
        return w;
    }

    constexpr Wide& operator+=(Wide b) noexcept
    {
        const auto previous = lo;
        lo += b.lo;
        hi += b.hi + (lo < previous ? 1u : 0u);
        return *this;
    }

    constexpr int sign() const noexcept
    {
        return (hi >> 63) != 0 ? -1 : ((lo | hi) != 0 ? 1 : 0);
    }
};

} // namespace cdt_detail

template<int GridFractionBits = 4>
class Cdt
{
public:
    // At least two raw bits are discarded. This also bounds coordinate
    // differences by 2^30, orientation/lift terms by 2^61 and the sum of
    // three in-circle products by 2^124: safely inside signed 128 bits.
    static_assert(GridFractionBits >= 0 && GridFractionBits <= Fixed::kFracBits - 2);
    static constexpr int kGridFractionBits = GridFractionBits;
    static constexpr std::int32_t kGridStepRaw = 1 << (Fixed::kFracBits - GridFractionBits);
    static constexpr Fixed kGridStep = Fixed::from_raw(kGridStepRaw);
    using VertexId = std::uint32_t;
    using TriangleId = std::uint32_t;
    using EdgeId = std::uint32_t;
    static constexpr std::uint32_t kInvalid = UINT32_MAX;

    struct GridPoint
    {
        std::int32_t x = 0, y = 0;
        bool operator==(const GridPoint&) const = default;
    };

    struct Triangle
    {
        // CCW. Edge i goes vertices[i] -> vertices[(i+1)%3].
        // Its twin is 3 * adjacent_triangle + adjacent_edge, or kInvalid
        // on the outer rectangle. Constraint bits agree on both twins.
        std::array<VertexId, 3> vertices{};
        std::array<EdgeId, 3> twins{kInvalid, kInvalid, kInvalid};
        std::uint8_t constraint_mask = 0;

        [[nodiscard]] bool constrained(unsigned edge) const noexcept
        {
            assert(edge < 3);
            return (constraint_mask & (1u << edge)) != 0;
        }
        bool operator==(const Triangle&) const = default;
    };

    enum class Error
    {
        none,
        not_initialized,
        coordinate_out_of_range,
        invalid_bounds,
        outside_bounds,
        invalid_vertex,
        collapsed_constraint,
        off_grid_intersection,
        capacity_exceeded,
        topology_error
    };

    struct InsertResult
    {
        Error error = Error::none;
        VertexId vertex = kInvalid;
        bool inserted = false; // false for an existing snapped vertex
        explicit operator bool() const noexcept { return error == Error::none; }
    };

    // Nearest grid point, ties away from zero, matching Fixed's rounding.
    // Rounding Fixed's positive maximum can leave its representable range;
    // reject that explicitly, never clamp or wrap collision coordinates.
    [[nodiscard]] static constexpr std::optional<CdtPoint> snap(CdtPoint p) noexcept
    {
        const auto x = quantize(p.x), y = quantize(p.y);
        if (!representable(x) || !representable(y)) return std::nullopt;
        return CdtPoint{world(x), world(y)};
    }

    // Bounds are snapped too. Four boundary vertices (CCW from lower left)
    // and two triangles are created; the four outside edges are constrained.
    // An error leaves the previous mesh intact, including on reset.
    [[nodiscard]] Error reset(CdtPoint lower, CdtPoint upper)
    {
        const auto lo = snap(lower), hi = snap(upper);
        if (!lo || !hi) return Error::coordinate_out_of_range;
        if (lo->x >= hi->x || lo->y >= hi->y) return Error::invalid_bounds;
        Cdt mesh;
        mesh.lower_ = grid(*lo);
        mesh.upper_ = grid(*hi);
        mesh.vertices_ = {{mesh.lower_.x, mesh.lower_.y}, {mesh.upper_.x, mesh.lower_.y},
                          {mesh.upper_.x, mesh.upper_.y}, {mesh.lower_.x, mesh.upper_.y}};
        mesh.triangles_ = {{{0, 1, 2}, {kInvalid, kInvalid, 3}, 3},
                           {{0, 2, 3}, {2, kInvalid, kInvalid}, 6}};
        *this = std::move(mesh);
        return Error::none;
    }

    [[nodiscard]] std::span<const GridPoint> vertices() const noexcept { return vertices_; }
    [[nodiscard]] std::span<const Triangle> triangles() const noexcept { return triangles_; }
    [[nodiscard]] CdtPoint position(VertexId v) const noexcept
    {
        assert(v < vertices_.size());
        return {world(vertices_[v].x), world(vertices_[v].y)};
    }

    [[nodiscard]] InsertResult insert_point(CdtPoint p)
    {
        if (triangles_.empty()) return {Error::not_initialized};
        const auto q = snap(p);
        if (!q) return {Error::coordinate_out_of_range};
        return insert_grid_point(grid(*q));
    }

    // Queries retain the full Fixed precision: a position just outside a wall
    // must not snap inside it. Points on an edge return the first incident face.
    [[nodiscard]] TriangleId locate(CdtPoint p) const noexcept
    {
        for (TriangleId f = 0; f < triangles_.size(); ++f)
        {
            const auto& t = triangles_[f];
            bool inside = true;
            for (unsigned i = 0; i < 3; ++i)
            {
                const auto a = vertices_[t.vertices[i]], b = vertices_[t.vertices[(i + 1) % 3]];
                const auto dx = std::int64_t{b.x} - a.x, dy = std::int64_t{b.y} - a.y;
                const auto px = std::int64_t{p.x.raw()} - std::int64_t{a.x} * kGridStepRaw;
                const auto py = std::int64_t{p.y.raw()} - std::int64_t{a.y} * kGridStepRaw;
                if (dx * py - dy * px < 0) { inside = false; break; }
            }
            if (inside) return f;
        }
        return kInvalid;
    }

    // Constraint operations are transactional. Collinear overlaps are
    // idempotent; existing points and on-grid intersections split segments.
    // Non-grid intersections return an error instead of bending either wall.
    [[nodiscard]] Error insert_constraint(VertexId a, VertexId b)
    {
        Cdt mesh = *this;
        const auto error = mesh.constrain(a, b);
        if (error == Error::none) *this = std::move(mesh);
        return error;
    }

    [[nodiscard]] Error insert_edge(CdtPoint a, CdtPoint b)
    {
        const std::array points{a, b};
        return insert_polyline(points);
    }

    // Closed loops constrain their boundary; they do not classify/remove
    // obstacle interiors. Winding does not imply walkability.
    [[nodiscard]] Error insert_polyline(std::span<const CdtPoint> points, bool closed = false)
    {
        if (triangles_.empty()) return Error::not_initialized;
        if (points.size() < (closed ? 3u : 2u)) return Error::collapsed_constraint;
        Cdt mesh = *this;
        std::vector<VertexId> ids;
        for (const auto p : points)
        {
            const auto result = mesh.insert_point(p);
            if (!result) return result.error;
            // Discard consecutive vertices collapsed by coarse quantization.
            if (ids.empty() || ids.back() != result.vertex) ids.push_back(result.vertex);
        }
        if (closed && ids.size() > 1 && ids.front() == ids.back()) ids.pop_back();
        if (ids.size() < (closed ? 3u : 2u)) return Error::collapsed_constraint;
        for (std::size_t i = 1; i < ids.size() + (closed ? 1u : 0u); ++i)
        {
            const auto error = mesh.constrain(ids[i - 1], ids[i % ids.size()]);
            if (error != Error::none) return error;
        }
        *this = std::move(mesh);
        return Error::none;
    }

private:
    using Triple = std::array<VertexId, 3>;
    using Key = std::pair<VertexId, VertexId>;
    struct Boundary { EdgeId twin; bool constrained; };
    std::vector<GridPoint> vertices_;
    std::vector<Triangle> triangles_;
    GridPoint lower_{}, upper_{};

    static constexpr std::int64_t quantize(Fixed p) noexcept
    {
        const std::int64_t r = p.raw();
        return r < 0 ? -((-r + kGridStepRaw / 2) / kGridStepRaw)
                     : (r + kGridStepRaw / 2) / kGridStepRaw;
    }
    static constexpr bool representable(std::int64_t p) noexcept
    {
        return p * kGridStepRaw >= INT32_MIN && p * kGridStepRaw <= INT32_MAX;
    }
    static constexpr Fixed world(std::int64_t p) noexcept
    {
        return Fixed::from_raw(static_cast<std::int32_t>(p * kGridStepRaw));
    }
    static GridPoint grid(CdtPoint p) noexcept
    {
        return {p.x.raw() / kGridStepRaw, p.y.raw() / kGridStepRaw};
    }
    static std::int64_t orient(GridPoint a, GridPoint b, GridPoint c) noexcept
    {
        return (std::int64_t{b.x} - a.x) * (std::int64_t{c.y} - a.y)
             - (std::int64_t{b.y} - a.y) * (std::int64_t{c.x} - a.x);
    }
    static bool on_segment(GridPoint a, GridPoint b, GridPoint p) noexcept
    {
        return orient(a, b, p) == 0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x)
                                  && p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y);
    }
    static bool opposite(std::int64_t a, std::int64_t b) noexcept
    {
        return (a < 0 && b > 0) || (a > 0 && b < 0);
    }
    static bool crosses(GridPoint a, GridPoint b, GridPoint c, GridPoint d) noexcept
    {
        return opposite(orient(a, b, c), orient(a, b, d))
            && opposite(orient(c, d, a), orient(c, d, b));
    }
    static int incircle(GridPoint a, GridPoint b, GridPoint c, GridPoint d) noexcept
    {
        const auto ax = std::int64_t{a.x} - d.x, ay = std::int64_t{a.y} - d.y;
        const auto bx = std::int64_t{b.x} - d.x, by = std::int64_t{b.y} - d.y;
        const auto cx = std::int64_t{c.x} - d.x, cy = std::int64_t{c.y} - d.y;
        auto sum = cdt_detail::Wide::product(ax * ax + ay * ay, bx * cy - by * cx);
        sum += cdt_detail::Wide::product(bx * bx + by * by, cx * ay - cy * ax);
        sum += cdt_detail::Wide::product(cx * cx + cy * cy, ax * by - ay * bx);
        return sum.sign();
    }

    VertexId start(EdgeId e) const noexcept { return triangles_[e / 3].vertices[e % 3]; }
    VertexId end(EdgeId e) const noexcept { return triangles_[e / 3].vertices[(e % 3 + 1) % 3]; }
    VertexId apex(EdgeId e) const noexcept { return triangles_[e / 3].vertices[(e % 3 + 2) % 3]; }
    EdgeId twin(EdgeId e) const noexcept { return triangles_[e / 3].twins[e % 3]; }
    bool constrained(EdgeId e) const noexcept { return triangles_[e / 3].constrained(e % 3); }
    void mark(EdgeId e)
    {
        triangles_[e / 3].constraint_mask |= static_cast<std::uint8_t>(1u << (e % 3));
        const auto t = twin(e);
        if (t != kInvalid)
            triangles_[t / 3].constraint_mask |= static_cast<std::uint8_t>(1u << (t % 3));
    }
    EdgeId find_edge(VertexId a, VertexId b) const noexcept
    {
        for (EdgeId e = 0; e < triangles_.size() * 3; ++e)
            if ((start(e) == a && end(e) == b) || (start(e) == b && end(e) == a)) return e;
        return kInvalid;
    }

    std::map<Key, Boundary> boundary(const std::vector<TriangleId>& removed) const
    {
        std::map<Key, Boundary> result;
        for (const auto f : removed)
            for (unsigned i = 0; i < 3; ++i)
            {
                const EdgeId e = 3 * f + i, t = twin(e);
                if (t == kInvalid || !std::binary_search(removed.begin(), removed.end(), t / 3))
                    result.emplace(Key{start(e), end(e)}, Boundary{t, constrained(e)});
            }
        return result;
    }

    // Replace a connected patch, reconnecting its outside twins and preserving
    // boundary constraints. IDs are reused, so vertices stay stable but face/
    // edge contents and all returned spans must be reacquired after mutations.
    std::vector<EdgeId> replace(std::vector<TriangleId> removed, const std::vector<Triple>& faces)
    {
        std::sort(removed.begin(), removed.end());
        const auto outside = boundary(removed);
        assert(faces.size() >= removed.size());
        while (removed.size() < faces.size())
        {
            removed.push_back(static_cast<TriangleId>(triangles_.size()));
            triangles_.push_back({});
        }
        std::map<Key, EdgeId> pending;
        std::vector<EdgeId> changed;
        for (std::size_t n = 0; n < faces.size(); ++n)
        {
            const auto f = removed[n];
            triangles_[f] = {faces[n]};
            assert(orient(vertices_[faces[n][0]], vertices_[faces[n][1]], vertices_[faces[n][2]]) > 0);
            for (unsigned i = 0; i < 3; ++i)
            {
                const EdgeId e = 3 * f + i;
                const Key key{start(e), end(e)};
                changed.push_back(e);
                if (const auto it = outside.find(key); it != outside.end())
                {
                    triangles_[f].twins[i] = it->second.twin;
                    if (it->second.twin != kInvalid)
                        triangles_[it->second.twin / 3].twins[it->second.twin % 3] = e;
                    if (it->second.constrained) mark(e);
                }
                else if (const auto a = vertices_[key.first], b = vertices_[key.second];
                         (a.x == b.x && (a.x == lower_.x || a.x == upper_.x))
                         || (a.y == b.y && (a.y == lower_.y || a.y == upper_.y)))
                {
                    // Inserting on the outside boundary replaces one edge by
                    // two; neither new edge has a twin in the patch.
                    mark(e);
                }
                else if (const auto match = pending.find({key.second, key.first}); match != pending.end())
                {
                    triangles_[f].twins[i] = match->second;
                    triangles_[match->second / 3].twins[match->second % 3] = e;
                    pending.erase(match);
                }
                else pending.emplace(key, e);
            }
        }
        assert(pending.empty());
        return changed;
    }

    void legalize(std::vector<EdgeId> pending)
    {
        while (!pending.empty())
        {
            const auto e = pending.back();
            pending.pop_back();
            const auto t = twin(e);
            if (t == kInvalid || constrained(e)) continue;
            const auto a = start(e), b = end(e), c = apex(e), d = apex(t);
            if (orient(vertices_[c], vertices_[d], vertices_[b]) <= 0
                || orient(vertices_[d], vertices_[c], vertices_[a]) <= 0) continue;
            if (incircle(vertices_[a], vertices_[b], vertices_[c], vertices_[d]) <= 0) continue;
            // Cocircular edges are left alone. Strict flips terminate without
            // an epsilon or a platform-dependent tie breaker.
            const auto changed = replace({e / 3, t / 3}, {{c, d, b}, {d, c, a}});
            pending.insert(pending.end(), changed.begin(), changed.end());
        }
    }

    InsertResult insert_grid_point(GridPoint p)
    {
        if (p.x < lower_.x || p.x > upper_.x || p.y < lower_.y || p.y > upper_.y)
            return {Error::outside_bounds};
        for (VertexId v = 0; v < vertices_.size(); ++v)
            if (vertices_[v] == p) return {Error::none, v, false};
        if (vertices_.size() >= kInvalid || triangles_.size() > kInvalid / 3 - 2)
            return {Error::capacity_exceeded};
        const auto f = locate({world(p.x), world(p.y)});
        if (f == kInvalid) return {Error::topology_error};
        const auto v = static_cast<VertexId>(vertices_.size());
        const auto face = triangles_[f];
        EdgeId edge = kInvalid;
        for (unsigned i = 0; i < 3; ++i)
            if (on_segment(vertices_[face.vertices[i]], vertices_[face.vertices[(i + 1) % 3]], p))
                edge = 3 * f + i;
        vertices_.push_back(p);
        std::vector<EdgeId> changed;
        if (edge == kInvalid)
        {
            const auto [a, b, c] = face.vertices;
            changed = replace({f}, {{a, b, v}, {b, c, v}, {c, a, v}});
        }
        else
        {
            const auto a = start(edge), b = end(edge), c = apex(edge), t = twin(edge);
            const bool wall = constrained(edge);
            std::vector<TriangleId> removed{f};
            std::vector<Triple> faces{{a, v, c}, {v, b, c}};
            if (t != kInvalid)
            {
                const auto d = apex(t);
                removed.push_back(t / 3);
                faces.push_back({b, v, d});
                faces.push_back({v, a, d});
            }
            changed = replace(std::move(removed), faces);
            if (wall) { mark(find_edge(a, v)); mark(find_edge(v, b)); }
        }
        legalize(std::move(changed));
        return {Error::none, v, true};
    }

    Error recover_edge(VertexId a, VertexId b)
    {
        if (const auto e = find_edge(a, b); e != kInvalid) { mark(e); return Error::none; }
        // Recover the segment by flipping crossing edges. Keep vertex pairs
        // in the FIFO: neighboring flips can change the meaning of edge IDs.
        // Non-convex pairs are deferred until adjacent flips make them convex;
        // new diagonals still crossing the constraint go to the back as well.
        // Unlike cavity ear clipping, this also handles corridors that touch
        // the same vertex more than once without losing a collision boundary.
        std::vector<Key> crossing;
        for (EdgeId e = 0; e < triangles_.size() * 3; ++e)
        {
            const auto t = twin(e);
            if (t == kInvalid || e > t) continue;
            if (!crosses(vertices_[a], vertices_[b], vertices_[start(e)], vertices_[end(e)])) continue;
            if (constrained(e)) return Error::topology_error;
            crossing.emplace_back(start(e), end(e));
        }
        const auto parameter = [&](Key edge) {
            auto n = orient(vertices_[edge.first], vertices_[edge.second], vertices_[a]);
            auto d = n - orient(vertices_[edge.first], vertices_[edge.second], vertices_[b]);
            if (d < 0) { n = -n; d = -d; }
            return std::pair{n, d};
        };
        std::sort(crossing.begin(), crossing.end(), [&](Key u, Key v) {
            const auto [un, ud] = parameter(u);
            const auto [vn, vd] = parameter(v);
            auto difference = cdt_detail::Wide::product(un, vd);
            difference += cdt_detail::Wide::product(-vn, ud);
            return difference.sign() < 0;
        });
        std::deque<Key> pending(crossing.begin(), crossing.end());
        std::vector<EdgeId> changed;
        // A defensive bound on the quadratic flip-recovery process. It turns
        // an internal topology bug into a transactional error, not a hang.
        const auto count = std::uint64_t{triangles_.size()} + 1;
        auto budget = 4 * count * count;
        std::size_t deferred = 0;
        while (!pending.empty())
        {
            if (budget-- == 0) return Error::topology_error;
            const auto key = pending.front();
            pending.pop_front();
            const auto e = find_edge(key.first, key.second);
            if (e == kInvalid || twin(e) == kInvalid || constrained(e)) return Error::topology_error;
            const auto t = twin(e), u = start(e), v = end(e), c = apex(e), d = apex(t);
            if (orient(vertices_[c], vertices_[d], vertices_[v]) <= 0
                || orient(vertices_[d], vertices_[c], vertices_[u]) <= 0)
            {
                pending.push_back(key);
                if (++deferred >= pending.size()) return Error::topology_error;
                continue;
            }
            deferred = 0;
            const auto flipped = replace({e / 3, t / 3}, {{c, d, v}, {d, c, u}});
            changed.insert(changed.end(), flipped.begin(), flipped.end());
            if (crosses(vertices_[a], vertices_[b], vertices_[c], vertices_[d])) pending.emplace_back(c, d);
        }
        const auto e = find_edge(a, b);
        if (e == kInvalid) return Error::topology_error;
        mark(e);
        legalize(std::move(changed));
        return Error::none;
    }

    Error constrain(VertexId a, VertexId b)
    {
        if (triangles_.empty()) return Error::not_initialized;
        if (a >= vertices_.size() || b >= vertices_.size()) return Error::invalid_vertex;
        if (a == b) return Error::collapsed_constraint;
        const auto pa = vertices_[a], pb = vertices_[b];
        std::vector<GridPoint> intersections;
        for (EdgeId e = 0; e < triangles_.size() * 3; ++e)
        {
            if (!constrained(e) || (twin(e) != kInvalid && e > twin(e))) continue;
            const auto c = vertices_[start(e)], d = vertices_[end(e)];
            if (!crosses(pa, pb, c, d)) continue;
            auto n = orient(c, d, pa);
            auto denominator = n - orient(c, d, pb);
            if (denominator < 0) { n = -n; denominator = -denominator; }
            const auto divisor = std::gcd(n, denominator);
            n /= divisor;
            denominator /= divisor;
            const auto dx = std::int64_t{pb.x} - pa.x, dy = std::int64_t{pb.y} - pa.y;
            // Reduced n/d has an integer intersection iff d divides both
            // deltas. Divide first: no wide division or rounded intersections.
            if (dx % denominator != 0 || dy % denominator != 0) return Error::off_grid_intersection;
            intersections.push_back({static_cast<std::int32_t>(pa.x + dx / denominator * n),
                                     static_cast<std::int32_t>(pa.y + dy / denominator * n)});
        }
        for (const auto p : intersections)
        {
            const auto result = insert_grid_point(p);
            if (!result) return result.error;
        }
        std::vector<VertexId> chain;
        for (VertexId v = 0; v < vertices_.size(); ++v)
            if (on_segment(pa, pb, vertices_[v])) chain.push_back(v);
        std::sort(chain.begin(), chain.end(), [&](VertexId u, VertexId v) {
            if (pa.x != pb.x) return pa.x < pb.x ? vertices_[u].x < vertices_[v].x : vertices_[u].x > vertices_[v].x;
            return pa.y < pb.y ? vertices_[u].y < vertices_[v].y : vertices_[u].y > vertices_[v].y;
        });
        for (std::size_t i = 1; i < chain.size(); ++i)
        {
            const auto error = recover_edge(chain[i - 1], chain[i]);
            if (error != Error::none) return error;
        }
        return Error::none;
    }
};

} // namespace rts
