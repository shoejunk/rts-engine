#include "editor/scene.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <stdexcept>

namespace rts::editor {
namespace {
std::optional<CdtPoint> input_point(Point p)
{
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0 || p.y < 0
        || p.x > LineDrawing::kWidth || p.y > LineDrawing::kHeight) return std::nullopt;
    return CdtPoint{Fixed::from_raw(static_cast<Fixed::Raw>(std::lround(p.x * Fixed::kRawOne))),
                    Fixed::from_raw(static_cast<Fixed::Raw>(std::lround(p.y * Fixed::kRawOne)))};
}
std::int64_t squared(CdtPoint a, CdtPoint b)
{
    const auto x = std::int64_t{a.x.raw()} - b.x.raw(), y = std::int64_t{a.y.raw()} - b.y.raw();
    return x * x + y * y;
}
} // namespace

Point display_point(CdtPoint point)
{
    return {static_cast<float>(point.x.to_double()), static_cast<float>(point.y.to_double())};
}

bool Scene::begin_line(Point point, float radius)
{
    return !playing_ && drawing_.begin(point, radius);
}
void Scene::preview_line(Point point, float radius)
{
    if (!playing_) drawing_.move(point, radius);
}
LineDrawing::Error Scene::finish_line(Point point, float radius)
{
    if (playing_ || !drawing_.preview()) return LineDrawing::Error::none;
    auto previous = drawing_;
    const auto result = drawing_.finish(point, radius);
    if (result == LineDrawing::Error::none) {
        const auto walls = constraint_walls(drawing_);
        for (const auto& unit : units_) {
            if (clear_position(unit.position, unit.radius, walls)) continue;
            drawing_ = std::move(previous);
            drawing_.cancel();
            // The geometry operation succeeded but the editor's unit-clearance
            // check failed. Its separate status is displayed by the caller.
            return LineDrawing::Error::invalid_vertex;
        }
        history_.push_back(Edit::line);
    }
    return result;
}

bool Scene::can_place_unit(Point point, int radius) const
{
    if (playing_ || radius < 2 || radius > 64) return false;
    const auto p = input_point(point);
    if (!p || !clear_position(*p, Fixed(radius), constraint_walls(drawing_))) return false;
    for (const auto& unit : units_) {
        const auto combined = std::int64_t{unit.radius.raw()} + Fixed(radius).raw();
        if (squared(*p, unit.position) < combined * combined) return false;
    }
    return true;
}
bool Scene::place_unit(Point point, int radius)
{
    if (!can_place_unit(point, radius)) return false;
    units_.push_back({*input_point(point), Fixed(radius), false, {}, 0});
    history_.push_back(Edit::unit);
    return true;
}
bool Scene::undo()
{
    if (playing_) return false;
    if (drawing_.preview()) { drawing_.cancel(); return true; }
    if (history_.empty()) return false;
    if (history_.back() == Edit::unit) units_.pop_back();
    else if (drawing_.undo() != LineDrawing::Error::none) return false;
    history_.pop_back();
    return true;
}
void Scene::set_playing(bool playing)
{
    if (playing == playing_) return;
    drawing_.cancel();
    playing_ = playing;
    if (playing) { playUnits_ = units_; walls_ = constraint_walls(drawing_); }
    else { playUnits_.clear(); walls_.clear(); }
}
std::size_t Scene::selected_count() const
{
    return static_cast<std::size_t>(std::count_if(playUnits_.begin(), playUnits_.end(), [](const Unit& unit) { return unit.selected; }));
}
void Scene::select(Point a, Point b, bool additive, bool click)
{
    if (!playing_ || !std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y)) return;
    if (!additive) for (auto& unit : playUnits_) unit.selected = false;
    if (click) {
        Unit* closest = nullptr;
        double best = 0;
        for (auto& unit : playUnits_) {
            const auto p = display_point(unit.position);
            const double x = p.x - b.x, y = p.y - b.y, distance = x * x + y * y;
            const double radius = unit.radius.to_double();
            if (distance <= radius * radius && (!closest || distance < best)) { closest = &unit; best = distance; }
        }
        if (closest) closest->selected = true;
        return;
    }
    for (auto& unit : playUnits_) {
        const auto p = display_point(unit.position);
        const double x = p.x - std::clamp(p.x, std::min(a.x, b.x), std::max(a.x, b.x));
        const double y = p.y - std::clamp(p.y, std::min(a.y, b.y), std::max(a.y, b.y));
        const double radius = unit.radius.to_double();
        if (x * x + y * y <= radius * radius) unit.selected = true;
    }
}
std::size_t Scene::move_selected(Point destination)
{
    const auto target = input_point(destination);
    const auto count = selected_count();
    if (!playing_ || !target || count == 0) return 0;
    std::int64_t sumX = 0, sumY = 0;
    for (const auto& unit : playUnits_) if (unit.selected) { sumX += unit.position.x.raw(); sumY += unit.position.y.raw(); }
    const CdtPoint center{Fixed::from_raw(static_cast<Fixed::Raw>(sumX / static_cast<std::int64_t>(count))),
                          Fixed::from_raw(static_cast<Fixed::Raw>(sumY / static_cast<std::int64_t>(count)))};
    std::size_t moving = 0;
    for (auto& unit : playUnits_) {
        if (!unit.selected) continue;
        // Translate the selection's formation to the clicked point so units
        // have distinct destinations. Their circles remain inside map bounds.
        const CdtPoint goal{std::clamp(target->x + unit.position.x - center.x, unit.radius, Fixed(LineDrawing::kWidth) - unit.radius),
                            std::clamp(target->y + unit.position.y - center.y, unit.radius, Fixed(LineDrawing::kHeight) - unit.radius)};
        unit.route = find_route(unit.position, goal, unit.radius, walls_);
        unit.waypoint = 0;
        if (!unit.route.empty()) ++moving;
    }
    return moving;
}
void Scene::tick()
{
    if (!playing_) return;
    for (auto& unit : playUnits_) {
        Fixed remaining(3); // 180 world units/sec at 60 Hz.
        while (remaining > Fixed(0) && unit.waypoint < unit.route.size()) {
            const auto goal = unit.route[unit.waypoint];
            const auto x = goal.x - unit.position.x, y = goal.y - unit.position.y;
            const auto distance = hypot(x, y);
            auto next = goal;
            if (distance > remaining) {
                next = {unit.position.x + mul_div(x, remaining, distance),
                        unit.position.y + mul_div(y, remaining, distance)};
            }
            // Never tunnel through a constraint, including fixed-point rounding
            // at a tangent. Units stop if the swept circle cannot advance.
            if (!clear_path(unit.position, next, unit.radius, walls_)) { unit.route.clear(); break; }
            unit.position = next;
            if (distance > remaining) break;
            remaining -= distance;
            ++unit.waypoint;
        }
    }
}
void Scene::save(const std::filesystem::path& path) const
{
    std::ofstream file;
    file.exceptions(std::ios::failbit | std::ios::badbit);
    file.imbue(std::locale::classic());
    file.open(path, std::ios::binary | std::ios::trunc);
    file << std::setprecision(17);
    file << "{\n  \"version\": 1,\n  \"width\": " << LineDrawing::kWidth << ",\n  \"height\": " << LineDrawing::kHeight << ",\n  \"lines\": [";
    bool first = true;
    for (const auto& line : drawing_.lines()) {
        file << (first ? "\n" : ",\n") << "    {\"start\": [" << line.start.x << ", " << line.start.y
             << "], \"end\": [" << line.end.x << ", " << line.end.y << "]}";
        first = false;
    }
    file << "\n  ],\n  \"units\": [";
    first = true;
    for (const auto& unit : units_) {
        file << (first ? "\n" : ",\n") << "    {\"x\": " << unit.position.x.to_double()
             << ", \"y\": " << unit.position.y.to_double() << ", \"radius\": " << unit.radius.to_double() << "}";
        first = false;
    }
    file << "\n  ]\n}\n";
    file.close();
}
void SceneFile::save(const Scene& scene) const
{
    if (!has_path()) throw std::logic_error("Choose a scene filename first.");
    scene.save(path_);
}
void SceneFile::save_as(const Scene& scene, const std::filesystem::path& path)
{
    scene.save(path);
    path_ = path; // A failed/canceled Save As never changes the current target.
}

} // namespace rts::editor
