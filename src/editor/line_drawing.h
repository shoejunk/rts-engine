#pragma once

#include "geometry/cdt.h"

#include <filesystem>
#include <optional>
#include <span>
#include <vector>

namespace rts::editor {

// Rendering/input coordinates. Committed endpoints come from the CDT lattice.
struct Point {
    float x = 0;
    float y = 0;
    bool operator==(const Point&) const = default;
};

struct Line {
    Point start;
    Point end;
    bool operator==(const Line&) const = default;
};

class LineDrawing {
public:
    using Mesh = Cdt<>;
    using Error = Mesh::Error;
    static constexpr int kWidth = 1200;
    static constexpr int kHeight = 700;

    LineDrawing();
    bool begin(Point point, float snapRadius = 10);
    void move(Point point, float snapRadius = 10);
    Error finish(Point point, float snapRadius = 10);
    void cancel();
    Error undo();

    // Reuse the closest vertex within the radius; otherwise snap to the CDT's
    // 1/16-unit lattice. Coordinates outside the bounded mesh are rejected.
    std::optional<Point> snap(Point point, float radius = 10) const;
    const Mesh& mesh() const { return mesh_; }
    Point vertex(Mesh::VertexId id) const;
    bool preview_valid() const { return previewValid_; }

    std::span<const Line> lines() const { return lines_; }
    const std::optional<Line>& preview() const { return preview_; }

    // Exports completed lines only, in drawing order. Throws on open/write/close
    // failure so the UI never reports a failed or partial write as successful.
    void save(const std::filesystem::path& path) const;

private:
    Mesh mesh_;
    std::vector<Line> lines_;
    std::optional<Line> preview_;
    bool previewValid_ = false;
};

} // namespace rts::editor
