#include "editor/line_drawing.h"

#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>

namespace rts::editor {

void LineDrawing::begin(Point point)
{
    preview_ = Line{point, point};
}

void LineDrawing::move(Point point)
{
    if (preview_) preview_->end = point;
}

void LineDrawing::finish(Point point)
{
    if (!preview_) return;
    // Use the release event, even when there was no preceding motion event.
    preview_->end = point;
    lines_.push_back(*preview_);
    preview_.reset();
}

void LineDrawing::cancel()
{
    preview_.reset();
}

void LineDrawing::undo()
{
    if (preview_) cancel();
    else if (!lines_.empty()) lines_.pop_back();
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
