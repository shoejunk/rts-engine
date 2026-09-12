#include "editor/line_drawing.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

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

Point canvas_point(float x, float y)
{
    return {x, y - kToolbarHeight};
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

void draw_line(SDL_Renderer* renderer, const Line& line, bool preview)
{
    if (preview) color(renderer, 255, 201, 92);
    else color(renderer, 104, 211, 255);
    SDL_RenderLine(renderer, line.start.x, line.start.y + kToolbarHeight,
                   line.end.x, line.end.y + kToolbarHeight);
    // Endpoint markers also make a zero-length line visible.
    for (const auto& point : {line.start, line.end}) {
        const SDL_FRect marker{point.x - 3, point.y + kToolbarHeight - 3, 6, 6};
        SDL_RenderFillRect(renderer, &marker);
    }
}

void render(SDL_Renderer* renderer, const LineDrawing& drawing, int width, int height,
            bool saving, const std::string& status)
{
    color(renderer, 20, 25, 34);
    SDL_RenderClear(renderer);
    const SDL_Rect canvas{0, kToolbarHeight, width, height - kToolbarHeight - kFooterHeight};
    SDL_SetRenderClipRect(renderer, &canvas);
    color(renderer, 32, 40, 52);
    for (int x = 0; x < width; x += 32)
        SDL_RenderLine(renderer, static_cast<float>(x), static_cast<float>(kToolbarHeight),
                       static_cast<float>(x), static_cast<float>(height - kFooterHeight));
    for (int y = kToolbarHeight; y < height - kFooterHeight; y += 32)
        SDL_RenderLine(renderer, 0, static_cast<float>(y), static_cast<float>(width), static_cast<float>(y));
    for (const auto& line : drawing.lines()) draw_line(renderer, line, false);
    if (drawing.preview()) draw_line(renderer, *drawing.preview(), true);
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
    text(renderer, 536, 24, std::to_string(drawing.lines().size()) + " lines");
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
    if (!SDL_CreateWindowAndRenderer("RTS Engine - Line Drawing", 1200, 800,
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
    std::string status = "Drag to draw. Esc cancels. Origin: canvas top-left.";
    std::string saveLocation = "lines.csv";

    const auto cancel_drag = [&] {
        drawing.cancel();
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
                    drawing.undo();
                    status = "Undid last line.";
                } else if (event.button.x >= 0 && event.button.x < static_cast<float>(width)
                           && event.button.y >= kToolbarHeight
                           && event.button.y < static_cast<float>(height - kFooterHeight)) {
                    drawing.begin(canvas_point(event.button.x, event.button.y));
                    SDL_CaptureMouse(true);
                }
            } else if (event.type == SDL_EVENT_MOUSE_MOTION) {
                drawing.move(canvas_point(event.motion.x, event.motion.y));
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT) {
                if (drawing.preview()) {
                    // Do not clamp: even an outside-window release keeps its exact endpoint.
                    drawing.finish(canvas_point(event.button.x, event.button.y));
                    status = "Line added. Save writes all completed lines.";
                }
                SDL_CaptureMouse(false);
            } else if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
                if (event.key.key == SDLK_ESCAPE) cancel_drag();
                else if (event.key.mod & SDL_KMOD_CTRL) {
                    if (event.key.key == SDLK_S) save();
                    else if (event.key.key == SDLK_Z) {
                        drawing.undo();
                        SDL_CaptureMouse(false);
                        status = "Undid last line or canceled drag.";
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
        render(renderer, drawing, width, height, saving, status);
        if (!vsync) SDL_Delay(16);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
