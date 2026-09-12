#include "editor/navigation.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>

namespace rts::editor {
namespace {
using I = std::int64_t;
I dx(CdtPoint a, CdtPoint b) { return I{a.x.raw()} - b.x.raw(); }
I dy(CdtPoint a, CdtPoint b) { return I{a.y.raw()} - b.y.raw(); }
I orient(CdtPoint a, CdtPoint b, CdtPoint c)
{
    return dx(b, a) * dy(c, a) - dy(b, a) * dx(c, a);
}
bool on_segment(CdtPoint p, CdtPoint a, CdtPoint b)
{
    return orient(a, b, p) == 0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x)
        && p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y);
}
bool intersects(CdtPoint a, CdtPoint b, CdtPoint c, CdtPoint d)
{
    const auto abC = orient(a, b, c), abD = orient(a, b, d);
    const auto cdA = orient(c, d, a), cdB = orient(c, d, b);
    return (((abC < 0 && abD > 0) || (abC > 0 && abD < 0))
        && ((cdA < 0 && cdB > 0) || (cdA > 0 && cdB < 0)))
        || (abC == 0 && on_segment(c, a, b)) || (abD == 0 && on_segment(d, a, b))
        || (cdA == 0 && on_segment(a, c, d)) || (cdB == 0 && on_segment(b, c, d));
}
bool near_segment(CdtPoint p, CdtPoint a, CdtPoint b, Fixed radius)
{
    const auto x = dx(p, a), y = dy(p, a), vx = dx(b, a), vy = dy(b, a);
    const I rr = I{radius.raw()} * radius.raw();
    const I length = vx * vx + vy * vy, dot = x * vx + y * vy;
    if (dot <= 0 || length == 0) return x * x + y * y < rr;
    if (dot >= length) {
        const auto ex = dx(p, b), ey = dy(p, b);
        return ex * ex + ey * ey < rr;
    }
    // Compare the rational squared distance exactly, without a division or
    // overflowing 64 bits. Wide is the CDT's portable integer accumulator.
    auto distance = cdt_detail::Wide::product(x * x + y * y, length);
    distance += cdt_detail::Wide::product(-dot, dot);
    distance += cdt_detail::Wide::product(-rr, length);
    return distance.sign() < 0;
}
bool in_bounds(CdtPoint p, Fixed radius)
{
    return radius > Fixed(0) && p.x >= radius && p.y >= radius
        && p.x <= Fixed(LineDrawing::kWidth) - radius && p.y <= Fixed(LineDrawing::kHeight) - radius;
}
} // namespace

std::vector<Wall> constraint_walls(const LineDrawing& drawing)
{
    std::vector<Wall> walls;
    const auto triangles = drawing.mesh().triangles();
    for (LineDrawing::Mesh::TriangleId face = 0; face < triangles.size(); ++face)
        for (unsigned edge = 0; edge < 3; ++edge) {
            const auto& triangle = triangles[face];
            if (!triangle.constrained(edge)) continue;
            const auto twin = triangle.twins[edge];
            if (twin != LineDrawing::Mesh::kInvalid && twin < 3 * face + edge) continue;
            walls.push_back({drawing.mesh().position(triangle.vertices[edge]),
                             drawing.mesh().position(triangle.vertices[(edge + 1) % 3])});
        }
    return walls;
}

bool clear_position(CdtPoint p, Fixed radius, std::span<const Wall> walls)
{
    if (!in_bounds(p, radius)) return false;
    for (const auto& wall : walls) if (near_segment(p, wall.a, wall.b, radius)) return false;
    return true;
}

bool clear_path(CdtPoint a, CdtPoint b, Fixed radius, std::span<const Wall> walls)
{
    if (!in_bounds(a, radius) || !in_bounds(b, radius)) return false;
    for (const auto& wall : walls)
        if (intersects(a, b, wall.a, wall.b)
            || near_segment(a, wall.a, wall.b, radius) || near_segment(b, wall.a, wall.b, radius)
            || near_segment(wall.a, a, b, radius) || near_segment(wall.b, a, b, radius)) return false;
    return true;
}

std::vector<CdtPoint> find_route(CdtPoint start, CdtPoint goal, Fixed radius, std::span<const Wall> walls)
{
    if (!clear_position(start, radius, walls) || !clear_position(goal, radius, walls)) return {};
    if (clear_path(start, goal, radius, walls)) return {goal};
    constexpr int step = 8, columns = LineDrawing::kWidth / step, rows = (LineDrawing::kHeight + step - 1) / step;
    constexpr int count = columns * rows;
    const auto point = [](int id) { return CdtPoint{Fixed(4 + (id % columns) * step), Fixed(4 + (id / columns) * step)}; };
    std::vector<int> cost(count, std::numeric_limits<int>::max()), parent(count, -1);
    std::vector<signed char> walkable(count, -1);
    std::vector<bool> closed(count);
    const auto is_free = [&](int id) {
        if (walkable[id] < 0) walkable[id] = clear_position(point(id), radius, walls) ? 1 : 0;
        return walkable[id] != 0;
    };
    const auto heuristic = [&](int id) {
        const auto p = point(id);
        return std::max(std::abs(p.x.to_int() - goal.x.to_int()), std::abs(p.y.to_int() - goal.y.to_int())) * 1000 / step;
    };
    using Candidate = std::tuple<int, int, int>; // f, g, stable node id
    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> pending;
    const int sx = start.x.to_int() / step, sy = start.y.to_int() / step;
    for (int y = sy - 1; y <= sy + 1; ++y)
        for (int x = sx - 1; x <= sx + 1; ++x) {
            if (x < 0 || y < 0 || x >= columns || y >= rows) continue;
            const int id = y * columns + x;
            if (!is_free(id) || !clear_path(start, point(id), radius, walls)) continue;
            cost[id] = hypot(point(id).x - start.x, point(id).y - start.y).to_int() * 1000 / step;
            pending.emplace(cost[id] + heuristic(id), cost[id], id);
        }
    int end = -1;
    constexpr std::array<std::pair<int, int>, 8> directions{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}}};
    while (!pending.empty()) {
        const auto [estimated, currentCost, id] = pending.top();
        (void)estimated;
        pending.pop();
        if (closed[id] || currentCost != cost[id]) continue;
        closed[id] = true;
        if (heuristic(id) <= 2000 && clear_path(point(id), goal, radius, walls)) { end = id; break; }
        for (const auto [ox, oy] : directions) {
            const int x = id % columns + ox, y = id / columns + oy;
            if (x < 0 || y < 0 || x >= columns || y >= rows) continue;
            const int next = y * columns + x;
            const int proposed = currentCost + (ox != 0 && oy != 0 ? 1414 : 1000);
            if (closed[next] || proposed >= cost[next] || !is_free(next)
                || !clear_path(point(id), point(next), radius, walls)) continue;
            cost[next] = proposed;
            parent[next] = id;
            pending.emplace(proposed + heuristic(next), proposed, next);
        }
    }
    if (end < 0) return {};
    std::vector<CdtPoint> route;
    for (int id = end; id >= 0; id = parent[id]) route.push_back(point(id));
    std::reverse(route.begin(), route.end());
    route.push_back(goal);
    std::vector<CdtPoint> smoothed;
    auto from = start;
    for (std::size_t next = 0; next < route.size();) {
        auto farthest = route.size() - 1;
        while (farthest > next && !clear_path(from, route[farthest], radius, walls)) --farthest;
        smoothed.push_back(route[farthest]);
        from = route[farthest];
        next = farthest + 1;
    }
    return smoothed;
}

} // namespace rts::editor
