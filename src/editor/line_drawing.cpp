#include "editor/line_drawing.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <stdexcept>

namespace rts::editor {

namespace {
CdtPoint cdt_point(Point point)
{
    // Called only after checking the small editor bounds. The explicit UI
    // conversion is the boundary between mouse floats and deterministic math.
    return {Fixed::from_raw(static_cast<Fixed::Raw>(std::lround(point.x * Fixed::kRawOne))),
            Fixed::from_raw(static_cast<Fixed::Raw>(std::lround(point.y * Fixed::kRawOne)))};
}

Point render_point(CdtPoint point)
{
    return {static_cast<float>(point.x.to_double()), static_cast<float>(point.y.to_double())};
}
} // namespace

LineDrawing::LineDrawing()
{
    if (mesh_.reset({Fixed(0), Fixed(0)}, {Fixed(kWidth), Fixed(kHeight)}) != Error::none)
        throw std::runtime_error("Could not initialize the CDT canvas.");
}

Point LineDrawing::vertex(Mesh::VertexId id) const
{
    return render_point(mesh_.position(id));
}

std::optional<Point> LineDrawing::snap(Point point, float radius) const
{
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
    std::optional<Point> closest;
    double distance = static_cast<double>(radius) * radius;
    for (Mesh::VertexId id = 0; id < mesh_.vertices().size(); ++id) {
        const auto candidate = vertex(id);
        const double dx = static_cast<double>(candidate.x) - point.x;
        const double dy = static_cast<double>(candidate.y) - point.y;
        const double squared = dx * dx + dy * dy;
        if (squared <= distance && (!closest || squared < distance)) {
            distance = squared;
            closest = candidate;
        }
    }
    if (closest) return closest;
    if (point.x < 0 || point.y < 0 || point.x > kWidth || point.y > kHeight)
        return std::nullopt;
    const auto quantized = Mesh::snap(cdt_point(point));
    return quantized ? std::optional{render_point(*quantized)} : std::nullopt;
}

bool LineDrawing::begin(Point point, float snapRadius)
{
    cancel();
    const auto snapped = snap(point, snapRadius);
    if (!snapped) return false;
    preview_ = Line{*snapped, *snapped};
    previewValid_ = true;
    return true;
}

void LineDrawing::move(Point point, float snapRadius)
{
    if (!preview_) return;
    const auto snapped = snap(point, snapRadius);
    previewValid_ = snapped.has_value();
    preview_->end = snapped.value_or(point);
}

LineDrawing::Error LineDrawing::finish(Point point, float snapRadius)
{
    if (!preview_) return Error::none;
    const auto start = preview_->start;
    // Snap the release itself, even if no motion event preceded it.
    const auto end = snap(point, snapRadius);
    cancel();
    if (!end) return Error::outside_bounds;
    const auto result = mesh_.insert_edge(cdt_point(start), cdt_point(*end));
    if (result == Error::none) lines_.push_back({start, *end});
    return result;
}

void LineDrawing::cancel()
{
    preview_.reset();
    previewValid_ = false;
}

LineDrawing::Error LineDrawing::undo()
{
    if (preview_) { cancel(); return Error::none; }
    if (lines_.empty()) return Error::none;
    // Replaying committed snapped endpoints removes both the constraint and
    // any vertices/intersections introduced by it, without changing older lines.
    Mesh rebuilt;
    auto result = rebuilt.reset({Fixed(0), Fixed(0)}, {Fixed(kWidth), Fixed(kHeight)});
    if (result != Error::none) return result;
    for (std::size_t i = 0; i + 1 < lines_.size(); ++i) {
        result = rebuilt.insert_edge(cdt_point(lines_[i].start), cdt_point(lines_[i].end));
        if (result != Error::none) return result;
    }
    mesh_ = std::move(rebuilt);
    lines_.pop_back();
    return Error::none;
}

void LineDrawing::save(const std::filesystem::path& path) const
{
    std::ofstream file;
    file.exceptions(std::ios::failbit | std::ios::badbit);
    file.imbue(std::locale::classic());
    file.open(path, std::ios::binary | std::ios::trunc);
    file << "start_x,start_y,end_x,end_y\n";
    file << std::setprecision(std::numeric_limits<float>::max_digits10);
    for (const auto& line : lines_)
        file << line.start.x << ',' << line.start.y << ','
             << line.end.x << ',' << line.end.y << '\n';
    file.close();
}

} // namespace rts::editor
