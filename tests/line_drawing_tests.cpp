#include "editor/line_drawing.h"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <locale>
#include <string>

using namespace rts::editor;

namespace {
int checks = 0, failures = 0;
#define CHECK(...) do { ++checks; if (!(__VA_ARGS__)) { ++failures; \
    std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (false)

std::string read(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

struct CommaDecimal : std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
};
} // namespace

int main()
{
    LineDrawing drawing;
    drawing.move({20, 30});
    drawing.finish({40, 50});
    CHECK(drawing.lines().empty()); // A release without a press draws nothing.

    drawing.begin({10.25f, 20.5f});
    drawing.move({100, 200});
    CHECK(drawing.lines().empty());
    CHECK(drawing.preview() == Line{{10.25f, 20.5f}, {100, 200}});
    drawing.finish({110.75f, 210.5f}); // Release differs from last motion.
    CHECK(!drawing.preview());
    CHECK(drawing.lines().size() == 1);
    CHECK(drawing.lines()[0] == Line{{10.25f, 20.5f}, {110.75f, 210.5f}});

    drawing.begin({50, 60});
    drawing.finish({-25, -30}); // No motion event and release outside canvas.
    CHECK(drawing.lines().size() == 2);
    CHECK(drawing.lines()[1] == Line{{50, 60}, {-25, -30}});
    drawing.begin({5, 6});
    drawing.finish({5, 6}); // Preserve both endpoints of a zero-length line.
    CHECK(drawing.lines().size() == 3);
    CHECK(drawing.lines()[2] == Line{{5, 6}, {5, 6}});
    drawing.undo();
    CHECK(drawing.lines().size() == 2);

    drawing.begin({1, 2});
    drawing.cancel(); // Escape/focus loss must not leave a dangling start.
    drawing.finish({3, 4});
    CHECK(drawing.lines().size() == 2);
    drawing.begin({1, 2});
    drawing.undo();
    CHECK(!drawing.preview());
    CHECK(drawing.lines().size() == 2);

    const auto folder = std::filesystem::temp_directory_path()
        / ("rts-lines-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(folder);
    const auto path = folder / std::filesystem::path(u8"line coordinates \u00e9.csv");
    const auto previousLocale = std::locale();
    try {
        // CSV must retain decimal dots even on systems using decimal commas.
        std::locale::global(std::locale(previousLocale, new CommaDecimal));
        drawing.begin({999, 999}); // An unfinished preview must never be saved.
        drawing.save(path);
        CHECK(read(path) == "start_x,start_y,end_x,end_y\n10.25,20.5,110.75,210.5\n50,60,-25,-30\n");
        drawing.cancel();
        drawing.undo();
        drawing.save(path); // Saving again replaces rather than appends.
        CHECK(read(path) == "start_x,start_y,end_x,end_y\n10.25,20.5,110.75,210.5\n");

        LineDrawing empty;
        empty.undo();
        empty.save(path);
        CHECK(read(path) == "start_x,start_y,end_x,end_y\n");
        bool failed = false;
        try { drawing.save(folder / "missing" / "lines.csv"); }
        catch (const std::ios_base::failure&) { failed = true; }
        CHECK(failed);
        CHECK(drawing.lines().size() == 1); // A failed save keeps the drawing.
    } catch (const std::exception& error) {
        CHECK(false);
        std::printf("Unexpected exception: %s\n", error.what());
    }
    std::locale::global(previousLocale);
    std::filesystem::remove(path);
    std::filesystem::remove(folder);
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
