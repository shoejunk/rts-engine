#pragma once

#include "editor/navigation.h"
#include <string>

namespace rts::editor {

Point display_point(CdtPoint point);

struct Unit {
    CdtPoint position;
    Fixed radius;
    bool selected = false;
    std::vector<CdtPoint> route;
    std::size_t waypoint = 0;
};

class Scene {
public:
    const LineDrawing& drawing() const { return drawing_; }
    bool begin_line(Point point, float radius);
    void preview_line(Point point, float radius);
    LineDrawing::Error finish_line(Point point, float radius);
    void cancel_line() { drawing_.cancel(); }
    bool place_unit(Point point, int radius);
    bool can_place_unit(Point point, int radius) const;
    bool undo();

    void set_playing(bool playing);
    bool playing() const { return playing_; }
    std::span<const Unit> units() const { return playing_ ? playUnits_ : units_; }
    std::size_t selected_count() const;
    void select(Point a, Point b, bool additive, bool click);
    std::size_t move_selected(Point destination);
    void tick(); // Exactly one 1/60-second simulation step, using Fixed math.

    // Save always writes the authored layout. Play is a disposable preview.
    void save(const std::filesystem::path& path) const;

private:
    enum class Edit { line, unit };
    LineDrawing drawing_;
    std::vector<Unit> units_, playUnits_;
    std::vector<Edit> history_;
    std::vector<Wall> walls_;
    bool playing_ = false;
};

class SceneFile {
public:
    bool has_path() const { return !path_.empty(); }
    const std::filesystem::path& path() const { return path_; }
    void save(const Scene& scene) const;
    void save_as(const Scene& scene, const std::filesystem::path& path);
private:
    std::filesystem::path path_;
};

} // namespace rts::editor
