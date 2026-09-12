#include "editor/line_drawing.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <mutex>
#include <string>

namespace {
using rts::editor::Line;
using rts::editor::LineDrawing;
using rts::editor::Point;

constexpr int kToolbarHeight = 64;
constexpr int kFooterHeight = 40;
constexpr SDL_FRect kSaveButton{16, 12, 256, 40};
constexpr SDL_FRect kUndoButton{288, 12, 224, 40};
constexpr SDL_DialogFileFilter kSaveFilters[] = {{"Line coordinates (CSV)", "csv"}};

bool contains(const SDL_FRect& rect, float x, float y)
{
    return x >= rect.x && y >= rect.y && x < rect.x + rect.w && y < rect.y + rect.h;
}

struct CanvasView {
    float scale;
    float x;
    float y;

    CanvasView(int width, int height)
    {
        const float availableWidth = static_cast<float>(std::max(1, width - 40));
        const float availableHeight = static_cast<float>(std::max(1, height - kToolbarHeight - kFooterHeight - 40));
        scale = std::min(availableWidth / LineDrawing::kWidth, availableHeight / LineDrawing::kHeight);
        x = (static_cast<float>(width) - LineDrawing::kWidth * scale) / 2;
        y = kToolbarHeight + (static_cast<float>(height - kToolbarHeight - kFooterHeight)
                              - LineDrawing::kHeight * scale) / 2;
    }
    Point to_canvas(float px, float py) const { return {(px - x) / scale, (py - y) / scale}; }
    Point to_window(Point point) const { return {x + point.x * scale, y + point.y * scale}; }
    float snap_radius() const { return 10 / scale; }
};

const char* constraint_status(LineDrawing::Error error)
{
    using Error = LineDrawing::Error;
    switch (error) {
    case Error::none: return "Constraint added. Triangulation updated.";
    case Error::collapsed_constraint: return "Choose two different CDT points.";
    case Error::off_grid_intersection: return "Crossing is off the CDT grid. Line not added.";
    case Error::outside_bounds: return "Endpoint is outside the CDT boundary.";
    default: return "CDT update failed. Drawing unchanged.";
    }
}

struct SaveDialog {
    std::mutex mutex;
    bool finished = false;
    std::string path;
    std::string error;
};

// SDL may invoke this on another thread. Only copy the result here; file I/O,
// rendering, and all access to the drawing stay on the main thread.
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

void draw_line(SDL_Renderer* renderer, const CanvasView& view, const Line& line,
               SDL_FColor tint, float thickness)
{
    const auto a = view.to_window(line.start), b = view.to_window(line.end);
    if (thickness <= 1) {
        SDL_SetRenderDrawColorFloat(renderer, tint.r, tint.g, tint.b, tint.a);
        SDL_RenderLine(renderer, a.x, a.y, b.x, b.y);
        return;
    }
    const float length = std::hypot(b.x - a.x, b.y - a.y);
    if (length == 0) return;
    const float nx = -(b.y - a.y) * thickness / (2 * length);
    const float ny = (b.x - a.x) * thickness / (2 * length);
    const SDL_Vertex vertices[] = {
        {{a.x + nx, a.y + ny}, tint, {}}, {{a.x - nx, a.y - ny}, tint, {}},
        {{b.x - nx, b.y - ny}, tint, {}}, {{b.x + nx, b.y + ny}, tint, {}}
    };
    constexpr int indices[] = {0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(renderer, nullptr, vertices, 4, indices, 6);
}

void point_marker(SDL_Renderer* renderer, const CanvasView& view, Point point, float size, bool filled)
{
    const auto position = view.to_window(point);
    const SDL_FRect marker{position.x - size / 2, position.y - size / 2, size, size};
    if (filled) SDL_RenderFillRect(renderer, &marker);
    else SDL_RenderRect(renderer, &marker);
}

void render(SDL_Renderer* renderer, const LineDrawing& drawing, int width, int height,
            bool saving, const std::string& status, const std::optional<Point>& hover)
{
    color(renderer, 20, 25, 34);
    SDL_RenderClear(renderer);
    const SDL_Rect canvas{0, kToolbarHeight, width, height - kToolbarHeight - kFooterHeight};
    SDL_SetRenderClipRect(renderer, &canvas);
    const CanvasView view(width, height);
    const auto triangles = drawing.mesh().triangles();
    // Draw each shared edge once, with all constraints over the faded edges.
    for (const bool constrained : {false, true}) {
        const SDL_FColor tint = constrained ? SDL_FColor{0.41f, 0.83f, 1, 1}
                                           : SDL_FColor{0.22f, 0.28f, 0.35f, 1};
        for (LineDrawing::Mesh::TriangleId face = 0; face < triangles.size(); ++face) {
            const auto& triangle = triangles[face];
            for (unsigned edge = 0; edge < 3; ++edge) {
                if (triangle.constrained(edge) != constrained) continue;
                const auto twin = triangle.twins[edge];
                if (twin != LineDrawing::Mesh::kInvalid && twin < 3 * face + edge) continue;
                draw_line(renderer, view, {drawing.vertex(triangle.vertices[edge]),
                          drawing.vertex(triangle.vertices[(edge + 1) % 3])}, tint, constrained ? 3.5f : 1);
            }
        }
    }
    color(renderer, 179, 203, 223);
    for (LineDrawing::Mesh::VertexId id = 0; id < drawing.mesh().vertices().size(); ++id)
        point_marker(renderer, view, drawing.vertex(id), 5, true);
    if (drawing.preview()) {
        const SDL_FColor tint = drawing.preview_valid() ? SDL_FColor{1, 0.79f, 0.36f, 1}
                                                       : SDL_FColor{1, 0.35f, 0.35f, 1};
        draw_line(renderer, view, *drawing.preview(), tint, 3.5f);
        SDL_SetRenderDrawColorFloat(renderer, tint.r, tint.g, tint.b, tint.a);
        point_marker(renderer, view, drawing.preview()->start, 9, false);
        point_marker(renderer, view, drawing.preview()->end, 9, false);
    } else if (hover) {
        color(renderer, 255, 201, 92);
        point_marker(renderer, view, *hover, 11, false);
    }
    SDL_SetRenderClipRect(renderer, nullptr);

    color(renderer, 30, 38, 50);
    const SDL_FRect toolbar{0, 0, static_cast<float>(width), static_cast<float>(kToolbarHeight)};
    const SDL_FRect footer{0, static_cast<float>(height - kFooterHeight), static_cast<float>(width),
                          static_cast<float>(kFooterHeight)};
    SDL_RenderFillRect(renderer, &toolbar);
    SDL_RenderFillRect(renderer, &footer);
    color(renderer, 45, 91, 126);
    SDL_RenderFillRect(renderer, &kSaveButton);
    color(renderer, 49, 61, 78);
    SDL_RenderFillRect(renderer, &kUndoButton);
    color(renderer, 234, 241, 249);
    text(renderer, 32, 24, saving ? "Saving..." : "Save [Ctrl+S]");
    text(renderer, 304, 24, "Undo [Ctrl+Z]");
    text(renderer, 536, 24, std::to_string(drawing.lines().size()) + " lines, "
                          + std::to_string(triangles.size()) + " tris");
    text(renderer, 16, static_cast<float>(height - 28), status);
    SDL_RenderPresent(renderer);
}
} // namespace

int main(int, char**)
{
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "RTS Engine", SDL_GetError(), nullptr);
        return 1;
    }
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    if (!SDL_CreateWindowAndRenderer("RTS Engine - CDT Constraints", 1200, 800,
                                     SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "RTS Engine", SDL_GetError(), nullptr);
        SDL_Quit();
        return 1;
    }
    SDL_SetWindowMinimumSize(window, 800, 400);
    const bool vsync = SDL_SetRenderVSync(renderer, 1);
    LineDrawing drawing;
    SaveDialog dialog;
    bool saving = false;
    bool quit = false;
    std::string status = "Drag to add constraints. Cyan: constrained edges.";
    std::string saveLocation = "lines.csv";
    std::optional<Point> hover;

    const auto cancel_drag = [&] {
        drawing.cancel();
        hover.reset();
        SDL_CaptureMouse(false);
    };
    const auto save = [&] {
        if (saving || drawing.preview()) return;
        {
            std::lock_guard lock(dialog.mutex);
            dialog.finished = false;
            dialog.path.clear();
            dialog.error.clear();
        }
        saving = true;
        status = "Choose a CSV file in the Save dialog.";
        SDL_ShowSaveFileDialog(save_dialog_result, &dialog, window, kSaveFilters, 1, saveLocation.c_str());
    };

    // Keep the dialog state and parent window alive until its callback returns,
    // including when a close request arrives while the native dialog is open.
    while (!quit || saving) {
        int width = 0, height = 0;
        SDL_GetWindowSize(window, &width, &height);
        const CanvasView view(width, height);
        // Render in window coordinates so DPI changes don't change saved units.
        SDL_SetRenderLogicalPresentation(renderer, width, height, SDL_LOGICAL_PRESENTATION_STRETCH);
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            SDL_ConvertEventToRenderCoordinates(renderer, &event);
            if (event.type == SDL_EVENT_QUIT || event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
                quit = true;
                cancel_drag();
            }
            if (event.type == SDL_EVENT_WINDOW_FOCUS_LOST) cancel_drag();
            if (saving || quit) continue;
            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT) {
                if (contains(kSaveButton, event.button.x, event.button.y)) save();
                else if (contains(kUndoButton, event.button.x, event.button.y)) {
                    const auto result = drawing.undo();
                    status = result == LineDrawing::Error::none ? "Undo complete. Triangulation updated."
                                                                : constraint_status(result);
                } else if (event.button.x >= 0 && event.button.x < static_cast<float>(width)
                           && event.button.y >= kToolbarHeight
                           && event.button.y < static_cast<float>(height - kFooterHeight)) {
                    if (drawing.begin(view.to_canvas(event.button.x, event.button.y), view.snap_radius()))
                        SDL_CaptureMouse(true);
                }
            } else if (event.type == SDL_EVENT_MOUSE_MOTION) {
                const auto point = view.to_canvas(event.motion.x, event.motion.y);
                hover = drawing.snap(point, view.snap_radius());
                drawing.move(point, view.snap_radius());
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT) {
                if (drawing.preview()) {
                    status = constraint_status(drawing.finish(view.to_canvas(event.button.x, event.button.y),
                                                               view.snap_radius()));
                }
                SDL_CaptureMouse(false);
            } else if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
                if (event.key.key == SDLK_ESCAPE) cancel_drag();
                else if (event.key.mod & SDL_KMOD_CTRL) {
                    if (event.key.key == SDLK_S) save();
                    else if (event.key.key == SDLK_Z) {
                        const auto result = drawing.undo();
                        SDL_CaptureMouse(false);
                        status = result == LineDrawing::Error::none ? "Undo complete. Triangulation updated."
                                                                    : constraint_status(result);
                    }
                }
            }
        }
        if (saving) {
            std::string path, error;
            {
                std::lock_guard lock(dialog.mutex);
                if (dialog.finished) {
                    saving = false;
                    path = dialog.path;
                    error = dialog.error;
                }
            }
            if (!saving) {
                if (!error.empty()) {
                    status = "Save dialog failed. Try Save again.";
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save failed", error.c_str(), window);
                } else if (path.empty()) status = "Save canceled.";
                else {
                    try {
                        // A UTF-8 path preserves non-ASCII Windows filenames.
                        drawing.save(std::filesystem::path(std::u8string(path.begin(), path.end())));
                        saveLocation = path;
                        status = "Saved " + std::to_string(drawing.lines().size()) + " lines to disk.";
                    } catch (const std::exception& errorMessage) {
                        status = "Save failed. Try another location.";
                        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Save failed", errorMessage.what(), window);
                    }
                }
            }
        }
        render(renderer, drawing, width, height, saving, status, hover);
        if (!vsync) SDL_Delay(16);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
