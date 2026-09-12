#pragma once

#include <filesystem>
#include <optional>
#include <span>
#include <vector>

namespace rts::editor {

// Canvas coordinates: origin at the top-left, x right, y down. These are
// editor/rendering values; they do not enter the fixed-point simulation.
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
    void begin(Point point);
    void move(Point point);
    void finish(Point point);
    void cancel();
    void undo();

    std::span<const Line> lines() const { return lines_; }
    const std::optional<Line>& preview() const { return preview_; }

    // Exports completed lines only, in drawing order. Throws on open/write/close
    // failure so the UI never reports a failed or partial write as successful.
    void save(const std::filesystem::path& path) const;

private:
    std::vector<Line> lines_;
    std::optional<Line> preview_;
};

} // namespace rts::editor
