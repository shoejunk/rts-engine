#pragma once

#include "editor/line_drawing.h"

namespace rts::editor {

struct Wall { CdtPoint a, b; };

std::vector<Wall> constraint_walls(const LineDrawing& drawing);
bool clear_position(CdtPoint position, Fixed radius, std::span<const Wall> walls);
bool clear_path(CdtPoint a, CdtPoint b, Fixed radius, std::span<const Wall> walls);
// Deterministic 8-unit search grid, followed by radius-aware line-of-sight
// smoothing. Returns an empty route when no traversable route was found.
std::vector<CdtPoint> find_route(CdtPoint start, CdtPoint goal, Fixed radius, std::span<const Wall> walls);

} // namespace rts::editor
