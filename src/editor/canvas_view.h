#pragma once

#include "editor/line_drawing.h"
#include <algorithm>
#include <cmath>

namespace rts::editor {

class CanvasView {
public:
    static constexpr int kToolbarHeight = 112;
    static constexpr int kFooterHeight = 36;

    void resize(int width, int height)
    {
        width_ = width;
        height_ = height;
        fit_ = std::min(static_cast<float>(std::max(1, width - 40)) / LineDrawing::kWidth,
                        static_cast<float>(std::max(1, height - kToolbarHeight - kFooterHeight - 40))
                        / LineDrawing::kHeight);
    }
    float scale() const { return fit_ * zoom_; }
    float zoom() const { return zoom_; }
    float snap_radius() const { return 10 / scale(); }
    bool contains(float x, float y) const
    {
        return x >= 0 && x < width_ && y >= kToolbarHeight && y < height_ - kFooterHeight;
    }
    Point to_canvas(float x, float y) const
    {
        return {center_.x + (x - width_ / 2.0f) / scale(),
                center_.y + (y - screen_center_y()) / scale()};
    }
    Point to_window(Point point) const
    {
        return {width_ / 2.0f + (point.x - center_.x) * scale(),
                screen_center_y() + (point.y - center_.y) * scale()};
    }
    void zoom_at(float wheel, float x, float y)
    {
        if (!contains(x, y) || !std::isfinite(wheel)) return;
        const auto anchor = to_canvas(x, y);
        zoom_ = std::clamp(zoom_ * std::pow(1.2f, std::clamp(wheel, -10.0f, 10.0f)), 0.25f, 8.0f);
        const auto shifted = to_canvas(x, y);
        center_.x += anchor.x - shifted.x;
        center_.y += anchor.y - shifted.y;
    }
    void reset() { zoom_ = 1; center_ = {LineDrawing::kWidth / 2.0f, LineDrawing::kHeight / 2.0f}; }

private:
    float screen_center_y() const { return kToolbarHeight + (height_ - kToolbarHeight - kFooterHeight) / 2.0f; }
    int width_ = 1200, height_ = 800;
    float fit_ = 1, zoom_ = 1;
    Point center_{LineDrawing::kWidth / 2.0f, LineDrawing::kHeight / 2.0f};
};

} // namespace rts::editor
