#include "editor/scene.h"
#include "editor/canvas_view.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <mutex>
#include <string>

namespace {
using namespace rts::editor;
constexpr int kToolbarHeight = CanvasView::kToolbarHeight;
constexpr int kFooterHeight = CanvasView::kFooterHeight;
constexpr SDL_FRect kSave{16, 12, 96, 36}, kSaveAs{124, 12, 176, 36}, kUndo{312, 12, 96, 36};
constexpr SDL_FRect kPlay{420, 12, 104, 36}, kFit{536, 12, 120, 36};
constexpr SDL_FRect kLines{16, 62, 104, 36}, kUnits{132, 62, 104, 36};
constexpr SDL_FRect kRadiusDown{428, 62, 36, 36}, kRadiusUp{476, 62, 36, 36};
constexpr SDL_DialogFileFilter kSaveFilters[] = {{"RTS scene (JSON)", "json"}};
enum class Tool { lines, units };
struct Selection { Point start, end, screenStart; bool additive = false; };

bool contains(const SDL_FRect& rect, float x, float y)
{
    return x >= rect.x && y >= rect.y && x < rect.x + rect.w && y < rect.y + rect.h;
}
const char* constraint_status(LineDrawing::Error error)
{
    using Error = LineDrawing::Error;
    switch (error) {
    case Error::none: return "Constraint added. Triangulation updated.";
    case Error::collapsed_constraint: return "Choose two different CDT points.";
    case Error::off_grid_intersection: return "Crossing is off the CDT grid. Line not added.";
    case Error::outside_bounds: return "Endpoint is outside the CDT boundary.";
    case Error::invalid_vertex: return "Constraint overlaps a unit. Line not added.";
    default: return "CDT update failed. Drawing unchanged.";
    }
}
struct SaveDialog {
    std::mutex mutex;
    bool finished = false;
    std::string path, error;
};
void SDLCALL save_dialog_result(void* userdata, const char* const* files, int)
{
    auto& dialog = *static_cast<SaveDialog*>(userdata);
    std::lock_guard lock(dialog.mutex);
    if (!files) {
        dialog.error = SDL_GetError();
        if (dialog.error.empty()) dialog.error = "Could not open the Save dialog.";
    } else if (files[0]) dialog.path = files[0];
    dialog.finished = true;
}
void color(SDL_Renderer* renderer, Uint8 r, Uint8 g, Uint8 b)
{
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);
}
void text(SDL_Renderer* renderer, float x, float y, const std::string& value)
{
    SDL_SetRenderScale(renderer, 2, 2);
    SDL_RenderDebugText(renderer, x / 2, y / 2, value.c_str());
    SDL_SetRenderScale(renderer, 1, 1);
}
void button(SDL_Renderer* renderer, const SDL_FRect& rect, const std::string& label, bool active = false, bool enabled = true)
{
    if (!enabled) color(renderer, 37, 44, 55);
    else if (active) color(renderer, 41, 110, 106);
    else color(renderer, 49, 70, 93);
    SDL_RenderFillRect(renderer, &rect);
    if (enabled) color(renderer, 234, 241, 249);
    else color(renderer, 102, 112, 124);
    text(renderer, rect.x + (rect.w - static_cast<float>(label.size()) * 16) / 2, rect.y + 10, label);
}
void draw_line(SDL_Renderer* renderer, const CanvasView& view, const Line& line, SDL_FColor tint, float thickness)
{
    const auto a = view.to_window(line.start), b = view.to_window(line.end);
    if (thickness <= 1) {
        SDL_SetRenderDrawColorFloat(renderer, tint.r, tint.g, tint.b, tint.a);
        SDL_RenderLine(renderer, a.x, a.y, b.x, b.y);
        return;
    }
    const float length = std::hypot(b.x - a.x, b.y - a.y);
    if (length == 0) return;
    const float nx = -(b.y - a.y) * thickness / (2 * length), ny = (b.x - a.x) * thickness / (2 * length);
    const SDL_Vertex vertices[] = {
        {{a.x + nx, a.y + ny}, tint, {}}, {{a.x - nx, a.y - ny}, tint, {}},
        {{b.x - nx, b.y - ny}, tint, {}}, {{b.x + nx, b.y + ny}, tint, {}}
    };
    constexpr int indices[] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(renderer, nullptr, vertices, 4, indices, 6);
}
void point_marker(SDL_Renderer* renderer, const CanvasView& view, Point point, float size, bool filled)
{
    const auto p = view.to_window(point);
    const SDL_FRect rect{p.x - size / 2, p.y - size / 2, size, size};
    if (filled) SDL_RenderFillRect(renderer, &rect);
    else SDL_RenderRect(renderer, &rect);
}
void circle(SDL_Renderer* renderer, const CanvasView& view, Point center, float radius, SDL_FColor tint, bool filled)
{
    const auto p = view.to_window(center);
    const float screenRadius = radius * view.scale();
    const int segments = std::clamp(static_cast<int>(screenRadius * 2), 24, 96);
    std::vector<SDL_FPoint> ring;
    std::vector<SDL_Vertex> vertices{{{p.x, p.y}, tint, {}}};
    std::vector<int> indices;
    for (int i = 0; i <= segments; ++i) {
        const float angle = static_cast<float>(i) * 6.28318530718f / static_cast<float>(segments);
        const SDL_FPoint edge{p.x + std::cos(angle) * screenRadius, p.y + std::sin(angle) * screenRadius};
        ring.push_back(edge);
        vertices.push_back({edge, tint, {}});
        if (i < segments) { indices.push_back(0); indices.push_back(i + 1); indices.push_back(i + 2); }
    }
    if (filled) SDL_RenderGeometry(renderer, nullptr, vertices.data(), static_cast<int>(vertices.size()), indices.data(), static_cast<int>(indices.size()));
    else {
        SDL_SetRenderDrawColorFloat(renderer, tint.r, tint.g, tint.b, tint.a);
        SDL_RenderLines(renderer, ring.data(), static_cast<int>(ring.size()));
    }
}
void render(SDL_Renderer* renderer, const Scene& scene, const CanvasView& view, int width, int height,
            bool saving, Tool tool, int radius, const std::string& status,
            const std::optional<Point>& hover, const std::optional<Selection>& selection)
{
    const auto& drawing = scene.drawing();
    color(renderer, 20, 25, 34);
    SDL_RenderClear(renderer);
    const SDL_Rect canvas{0, kToolbarHeight, width, height - kToolbarHeight - kFooterHeight};
    SDL_SetRenderClipRect(renderer, &canvas);
    const auto triangles = drawing.mesh().triangles();
    for (const bool constrained : {false, true}) {
        const SDL_FColor tint = constrained ? SDL_FColor{0.41f, 0.83f, 1, 1} : SDL_FColor{0.22f, 0.28f, 0.35f, 1};
        for (LineDrawing::Mesh::TriangleId face = 0; face < triangles.size(); ++face) {
            const auto& triangle = triangles[face];
            for (unsigned edge = 0; edge < 3; ++edge) {
                if (triangle.constrained(edge) != constrained) continue;
                const auto twin = triangle.twins[edge];
                if (twin != LineDrawing::Mesh::kInvalid && twin < 3 * face + edge) continue;
                draw_line(renderer, view, {drawing.vertex(triangle.vertices[edge]), drawing.vertex(triangle.vertices[(edge + 1) % 3])}, tint, constrained ? 3.5f : 1);
            }
        }
    }
    color(renderer, 179, 203, 223);
    for (LineDrawing::Mesh::VertexId id = 0; id < drawing.mesh().vertices().size(); ++id)
        point_marker(renderer, view, drawing.vertex(id), 5, true);
    for (const auto& unit : scene.units()) {
        const auto position = display_point(unit.position);
        if (unit.selected && unit.waypoint < unit.route.size()) {
            auto from = position;
            for (std::size_t i = unit.waypoint; i < unit.route.size(); ++i) {
                const auto to = display_point(unit.route[i]);
                draw_line(renderer, view, {from, to}, {0.22f, 0.46f, 0.31f, 1}, 1);
                from = to;
            }
            circle(renderer, view, from, 4 / view.scale(), {0.5f, 1, 0.6f, 1}, false);
        }
        const auto unitRadius = static_cast<float>(unit.radius.to_double());
        circle(renderer, view, position, unitRadius, {0.33f, 0.75f, 0.48f, 1}, true);
        if (unit.selected) circle(renderer, view, position, unitRadius + 3 / view.scale(), {0.8f, 1, 0.85f, 1}, false);
    }
    if (!scene.playing() && drawing.preview()) {
        const SDL_FColor tint = drawing.preview_valid() ? SDL_FColor{1, 0.79f, 0.36f, 1} : SDL_FColor{1, 0.35f, 0.35f, 1};
        draw_line(renderer, view, *drawing.preview(), tint, 3.5f);
        SDL_SetRenderDrawColorFloat(renderer, tint.r, tint.g, tint.b, tint.a);
        point_marker(renderer, view, drawing.preview()->start, 9, false);
        point_marker(renderer, view, drawing.preview()->end, 9, false);
    } else if (!scene.playing() && hover) {
        if (tool == Tool::units) circle(renderer, view, *hover, static_cast<float>(radius),
            scene.can_place_unit(*hover, radius) ? SDL_FColor{0.6f, 1, 0.65f, 1} : SDL_FColor{1, 0.35f, 0.35f, 1}, false);
        else { color(renderer, 255, 201, 92); point_marker(renderer, view, *hover, 11, false); }
    }
    if (selection) {
        const auto a = view.to_window(selection->start), b = view.to_window(selection->end);
        const SDL_FRect rect{std::min(a.x, b.x), std::min(a.y, b.y), std::abs(a.x - b.x), std::abs(a.y - b.y)};
        SDL_SetRenderDrawColor(renderer, 110, 220, 160, 40);
        SDL_RenderFillRect(renderer, &rect);
        color(renderer, 145, 235, 180);
        SDL_RenderRect(renderer, &rect);
    }
    SDL_SetRenderClipRect(renderer, nullptr);
    color(renderer, 30, 38, 50);
    const SDL_FRect toolbar{0, 0, static_cast<float>(width), static_cast<float>(kToolbarHeight)};
    const SDL_FRect footer{0, static_cast<float>(height - kFooterHeight), static_cast<float>(width), static_cast<float>(kFooterHeight)};
    SDL_RenderFillRect(renderer, &toolbar);
    SDL_RenderFillRect(renderer, &footer);
    button(renderer, kSave, "Save", false, !saving);
    button(renderer, kSaveAs, "Save As...", false, !saving);
    button(renderer, kUndo, "Undo", false, !saving && !scene.playing());
    button(renderer, kPlay, scene.playing() ? "Stop" : "Play", scene.playing(), !saving);
    button(renderer, kFit, "Fit Map", false, !saving);
    button(renderer, kLines, "Lines", tool == Tool::lines, !saving && !scene.playing());
    button(renderer, kUnits, "Units", tool == Tool::units, !saving && !scene.playing());
    button(renderer, kRadiusDown, "-", false, !saving && !scene.playing() && radius > 2);
    button(renderer, kRadiusUp, "+", false, !saving && !scene.playing() && radius < 64);
    color(renderer, 230, 238, 246);
    text(renderer, 256, 74, "Radius: " + std::to_string(radius));
    text(renderer, 680, 22, scene.playing() ? "PLAY | " + std::to_string(scene.selected_count()) + " selected" : "EDIT | Wheel to zoom");
    text(renderer, 536, 74, std::to_string(drawing.lines().size()) + " lines / " + std::to_string(scene.units().size())
         + " units / " + std::to_string(static_cast<int>(std::lround(view.zoom() * 100))) + "%");
    text(renderer, 16, static_cast<float>(height - 25), status);
    SDL_RenderPresent(renderer);
}
} // namespace

int main(int, char**)
{
    if (!SDL_Init(SDL_INIT_VIDEO)) { SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "RTS Engine", SDL_GetError(), nullptr); return 1; }
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    if (!SDL_CreateWindowAndRenderer("RTS Engine - Scene Editor", 1200, 800, SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "RTS Engine", SDL_GetError(), nullptr);
        SDL_Quit();
        return 1;
    }
    SDL_SetWindowMinimumSize(window, 1100, 480);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    const bool vsync = SDL_SetRenderVSync(renderer, 1);
    Scene scene;
    SceneFile sceneFile;
    CanvasView view;
    SaveDialog dialog;
    bool saving = false, quit = false, dirty = false;
    Tool tool = Tool::lines;
    int radius = 12;
    std::string status = "Draw constraints, or choose Units to place circles.";
    std::string saveLocation = "scene.json";
    std::optional<Point> hover;
    std::optional<Selection> selection;
    double previousTime = static_cast<double>(SDL_GetTicksNS()) / 1.0e9, accumulated = 0;
    const auto cancel_drag = [&] { scene.cancel_line(); hover.reset(); selection.reset(); SDL_CaptureMouse(false); };
    const auto save_success = [&] { dirty = false; status = "Saved constraints and unit placements."; };
    const auto save_error = [&](const std::exception& error) {
        status = "Save failed. Try another location.";
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save failed", error.what(), window);
    };
    const auto save = [&](bool saveAs) {
        if (saving) return;
        cancel_drag();
        if (!saveAs && sceneFile.has_path()) {
            try { sceneFile.save(scene); save_success(); } catch (const std::exception& error) { save_error(error); }
            return;
        }
        {
            std::lock_guard lock(dialog.mutex);
            dialog.finished = false; dialog.path.clear(); dialog.error.clear();
        }
        saving = true;
        status = "Choose a scene file in the Save dialog.";
        SDL_ShowSaveFileDialog(save_dialog_result, &dialog, window, kSaveFilters, 1, saveLocation.c_str());
    };
    const auto toggle_play = [&] {
        cancel_drag();
        scene.set_playing(!scene.playing());
        accumulated = 0;
        status = scene.playing() ? "Drag to select. Shift adds. Right-click to move." : "Edit mode. Unit placements restored.";
    };
    const auto undo = [&] {
        if (scene.undo()) { dirty = true; status = "Undid last edit."; }
        SDL_CaptureMouse(false);
    };
    while (!quit || saving) {
        int width = 0, height = 0;
        SDL_GetWindowSize(window, &width, &height);
        view.resize(width, height);
        SDL_SetRenderLogicalPresentation(renderer, width, height, SDL_LOGICAL_PRESENTATION_STRETCH);
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_WINDOW_RESIZED) {
                width = event.window.data1; height = event.window.data2;
                view.resize(width, height);
                SDL_SetRenderLogicalPresentation(renderer, width, height, SDL_LOGICAL_PRESENTATION_STRETCH);
            }
            SDL_ConvertEventToRenderCoordinates(renderer, &event);
            if (event.type == SDL_EVENT_QUIT || event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) { quit = true; cancel_drag(); }
            if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) cancel_drag();
            if (saving || quit) continue;
            if (event.type == SDL_EVENT_MOUSE_WHEEL) {
                const float direction = event.wheel.direction == SDL_MOUSEWHEEL_FLIPPED ? -1.0f : 1.0f;
                view.zoom_at(event.wheel.y * direction, event.wheel.mouse_x, event.wheel.mouse_y);
                const auto point = view.to_canvas(event.wheel.mouse_x, event.wheel.mouse_y);
                hover = view.contains(event.wheel.mouse_x, event.wheel.mouse_y) ? std::optional{point} : std::nullopt;
                if (tool == Tool::lines && hover) hover = scene.drawing().snap(point, view.snap_radius());
                scene.preview_line(point, view.snap_radius());
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT) {
                const float x = event.button.x, y = event.button.y;
                if (contains(kSave, x, y)) save(false);
                else if (contains(kSaveAs, x, y)) save(true);
                else if (contains(kPlay, x, y)) toggle_play();
                else if (contains(kFit, x, y)) { cancel_drag(); view.reset(); }
                else if (!scene.playing() && contains(kUndo, x, y)) undo();
                else if (!scene.playing() && (contains(kLines, x, y) || contains(kUnits, x, y))) {
                    cancel_drag(); tool = contains(kLines, x, y) ? Tool::lines : Tool::units;
                    status = tool == Tool::units ? "Click to place units. +/- or [ ] changes new-unit radius." : "Drag to add snapped CDT constraints.";
                } else if (!scene.playing() && contains(kRadiusDown, x, y)) radius = std::max(2, radius - 2);
                else if (!scene.playing() && contains(kRadiusUp, x, y)) radius = std::min(64, radius + 2);
                else if (view.contains(x, y)) {
                    const auto point = view.to_canvas(x, y);
                    if (scene.playing()) {
                        selection = Selection{point, point, {x, y}, (SDL_GetModState() & SDL_KMOD_SHIFT) != 0};
                        SDL_CaptureMouse(true);
                    } else if (tool == Tool::units) {
                        if (scene.place_unit(point, radius)) { dirty = true; status = "Unit placed. Radius applies to the next unit."; }
                        else status = "Unit must fit inside the map, clear of walls and units.";
                    } else if (scene.begin_line(point, view.snap_radius())) SDL_CaptureMouse(true);
                }
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_RIGHT
                       && scene.playing() && view.contains(event.button.x, event.button.y)) {
                selection.reset(); SDL_CaptureMouse(false);
                const auto moved = scene.move_selected(view.to_canvas(event.button.x, event.button.y));
                status = "Moving " + std::to_string(moved) + " / " + std::to_string(scene.selected_count()) + " selected units.";
                if (moved < scene.selected_count()) status += " Some goals blocked.";
            } else if (event.type == SDL_EVENT_MOUSE_MOTION) {
                const auto point = view.to_canvas(event.motion.x, event.motion.y);
                hover = view.contains(event.motion.x, event.motion.y) ? std::optional{point} : std::nullopt;
                if (tool == Tool::lines && hover) hover = scene.drawing().snap(point, view.snap_radius());
                scene.preview_line(point, view.snap_radius());
                if (selection) selection->end = point;
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT) {
                const auto point = view.to_canvas(event.button.x, event.button.y);
                if (scene.playing() && selection) {
                    const bool click = std::hypot(event.button.x - selection->screenStart.x, event.button.y - selection->screenStart.y) < 4;
                    scene.select(selection->start, point, selection->additive, click);
                    selection.reset();
                    status = std::to_string(scene.selected_count()) + " units selected. Right-click to move.";
                } else if (!scene.playing() && scene.drawing().preview()) {
                    const auto result = scene.finish_line(point, view.snap_radius());
                    if (result == LineDrawing::Error::none) dirty = true;
                    status = constraint_status(result);
                }
                SDL_CaptureMouse(false);
            } else if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
                if (event.key.key == SDLK_ESCAPE) cancel_drag();
                else if (event.key.mod & SDL_KMOD_CTRL) {
                    if (event.key.key == SDLK_S) save((event.key.mod & SDL_KMOD_SHIFT) != 0);
                    else if (event.key.key == SDLK_Z && !scene.playing()) undo();
                } else if (event.key.key == SDLK_SPACE) toggle_play();
                else if (event.key.key == SDLK_HOME) { cancel_drag(); view.reset(); }
                else if (!scene.playing() && event.key.key == SDLK_LEFTBRACKET) radius = std::max(2, radius - 1);
                else if (!scene.playing() && event.key.key == SDLK_RIGHTBRACKET) radius = std::min(64, radius + 1);
            }
        }
        if (saving) {
            std::string path, error;
            {
                std::lock_guard lock(dialog.mutex);
                if (dialog.finished) { saving = false; path = dialog.path; error = dialog.error; }
            }
            if (!saving) {
                if (!error.empty()) { status = "Save dialog failed."; SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save failed", error.c_str(), window); }
                else if (path.empty()) status = "Save canceled.";
                else try {
                    sceneFile.save_as(scene, std::filesystem::path(std::u8string(path.begin(), path.end())));
                    saveLocation = path;
                    save_success();
                } catch (const std::exception& errorMessage) { save_error(errorMessage); }
            }
        }
        const double now = static_cast<double>(SDL_GetTicksNS()) / 1.0e9;
        if (scene.playing() && !saving && !quit) {
            accumulated += std::clamp(now - previousTime, 0.0, 0.1);
            while (accumulated >= 1.0 / 60.0) { scene.tick(); accumulated -= 1.0 / 60.0; }
        } else accumulated = 0;
        previousTime = now;
        const auto filename = sceneFile.has_path() ? sceneFile.path().filename().u8string() : u8"Untitled";
        const std::string title = "RTS Engine - " + std::string(filename.begin(), filename.end()) + (dirty ? " *" : "") + (scene.playing() ? " [Play]" : " [Edit]");
        SDL_SetWindowTitle(window, title.c_str());
        render(renderer, scene, view, width, height, saving, tool, radius, status, hover, selection);
        if (!vsync) SDL_Delay(16);
    }
    SDL_DestroyRenderer(renderer); SDL_DestroyWindow(window); SDL_Quit();
    return 0;
}
