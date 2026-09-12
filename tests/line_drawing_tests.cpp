#include "editor/line_drawing.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <limits>
#include <locale>
#include <string>

using namespace rts::editor;
using Mesh = LineDrawing::Mesh;
using Error = LineDrawing::Error;

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

bool same_mesh(const Mesh& a, const Mesh& b)
{
    return std::ranges::equal(a.vertices(), b.vertices())
        && std::ranges::equal(a.triangles(), b.triangles());
}

bool has_edge(const LineDrawing& drawing, Point a, Point b, bool constrained)
{
    for (const auto& triangle : drawing.mesh().triangles())
        for (unsigned edge = 0; edge < 3; ++edge) {
            const auto start = drawing.vertex(triangle.vertices[edge]);
            const auto end = drawing.vertex(triangle.vertices[(edge + 1) % 3]);
            if (((start == a && end == b) || (start == b && end == a))
                && triangle.constrained(edge) == constrained) return true;
        }
    return false;
}
} // namespace

int main()
{
    LineDrawing drawing;
    const auto initial = drawing.mesh();
    CHECK(initial.vertices().size() == 4);
    CHECK(initial.triangles().size() == 2);
    CHECK(has_edge(drawing, {0, 0}, {1200, 700}, false));
    CHECK(has_edge(drawing, {0, 0}, {1200, 0}, true));
    drawing.move({20, 30});
    CHECK(drawing.finish({40, 50}) == Error::none);
    CHECK(drawing.lines().empty()); // Release without a press draws nothing.

    CHECK(drawing.begin({100.02f, 100.09f}));
    drawing.move({200, 250});
    CHECK(drawing.preview() == Line{{100, 100.0625f}, {200, 250}});
    CHECK(same_mesh(initial, drawing.mesh())); // Preview never edits the mesh.
    CHECK(drawing.finish({300.03125f, 100.09f}) == Error::none);
    CHECK(!drawing.preview());
    CHECK(drawing.lines().size() == 1);
    CHECK(drawing.lines()[0] == Line{{100, 100.0625f}, {300.0625f, 100.0625f}});
    CHECK(has_edge(drawing, {100, 100.0625f}, {300.0625f, 100.0625f}, true));
    CHECK(drawing.mesh().vertices().size() == 6);
    CHECK(drawing.mesh().triangles().size() == 6);
    const auto firstMesh = drawing.mesh();

    // Both press and release reuse nearby existing vertices, without duplicates.
    CHECK(drawing.begin({103, 102}));
    CHECK(drawing.preview()->start == Point{100, 100.0625f});
    CHECK(drawing.finish({303, 103}) == Error::none);
    CHECK(drawing.lines().size() == 2);
    CHECK(same_mesh(firstMesh, drawing.mesh()));
    CHECK(drawing.undo() == Error::none);
    CHECK(same_mesh(firstMesh, drawing.mesh())); // Undo duplicate keeps older wall.

    CHECK(drawing.begin({103, 102}));
    CHECK(drawing.finish({400.5f, 300.03125f}) == Error::none); // No motion needed.
    CHECK(drawing.lines().size() == 2);
    CHECK(drawing.lines()[1] == Line{{100, 100.0625f}, {400.5f, 300.0625f}});
    CHECK(drawing.mesh().vertices().size() == 7);
    const auto twoLinesMesh = drawing.mesh();

    drawing.begin({500, 500});
    CHECK(drawing.finish({500, 500}) == Error::collapsed_constraint);
    CHECK(same_mesh(twoLinesMesh, drawing.mesh()));
    drawing.begin({500, 500});
    drawing.move({-25, -30});
    CHECK(!drawing.preview_valid());
    CHECK(drawing.finish({-25, -30}) == Error::outside_bounds);
    CHECK(same_mesh(twoLinesMesh, drawing.mesh()));
    CHECK(drawing.lines().size() == 2);
    CHECK(!drawing.begin({-25, -30}));
    CHECK(!drawing.snap({std::numeric_limits<float>::quiet_NaN(), 0}));
    CHECK(!drawing.snap({std::numeric_limits<float>::max(), 0}));

    drawing.begin({500, 500});
    drawing.cancel(); // Escape/focus loss must not leave a dangling start.
    drawing.finish({600, 600});
    CHECK(same_mesh(twoLinesMesh, drawing.mesh()));
    drawing.begin({500, 500});
    CHECK(drawing.undo() == Error::none);
    CHECK(!drawing.preview());
    CHECK(drawing.lines().size() == 2);
    CHECK(same_mesh(twoLinesMesh, drawing.mesh()));

    LineDrawing crossing;
    crossing.begin({100, 100});
    CHECK(crossing.finish({300, 300}) == Error::none);
    const auto beforeCross = crossing.mesh();
    crossing.begin({100, 201});
    CHECK(crossing.finish({300, 100}) == Error::off_grid_intersection);
    CHECK(crossing.lines().size() == 1);
    CHECK(same_mesh(beforeCross, crossing.mesh())); // Failed insert rolls back endpoints too.
    crossing.begin({100, 300});
    CHECK(crossing.finish({300, 100}) == Error::none);
    CHECK(crossing.lines().size() == 2);
    CHECK(has_edge(crossing, {100, 100}, {200, 200}, true));
    CHECK(has_edge(crossing, {300, 300}, {200, 200}, true));
    CHECK(has_edge(crossing, {100, 300}, {200, 200}, true));
    CHECK(has_edge(crossing, {300, 100}, {200, 200}, true));
    CHECK(crossing.snap({202, 203}) == Point{200, 200}); // Intersection is a snap target.
    CHECK(crossing.undo() == Error::none);
    CHECK(same_mesh(beforeCross, crossing.mesh()));
    CHECK(crossing.undo() == Error::none);
    CHECK(same_mesh(initial, crossing.mesh()));

    const auto folder = std::filesystem::temp_directory_path()
        / ("rts-lines-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(folder);
    const auto path = folder / std::filesystem::path(u8"line coordinates \u00e9.csv");
    const auto previousLocale = std::locale();
    try {
        std::locale::global(std::locale(previousLocale, new CommaDecimal));
        drawing.begin({999, 600}); // Unfinished preview must not be saved.
        drawing.save(path);
        CHECK(read(path) == "start_x,start_y,end_x,end_y\n100,100.0625,300.0625,100.0625\n100,100.0625,400.5,300.0625\n");
        drawing.cancel();
        CHECK(drawing.undo() == Error::none);
        CHECK(same_mesh(firstMesh, drawing.mesh()));
        drawing.save(path);
        CHECK(read(path) == "start_x,start_y,end_x,end_y\n100,100.0625,300.0625,100.0625\n");

        LineDrawing empty;
        empty.undo();
        empty.save(path);
        CHECK(read(path) == "start_x,start_y,end_x,end_y\n"); // Excludes mesh boundary/diagonals.
        bool failed = false;
        try { drawing.save(folder / "missing" / "lines.csv"); }
        catch (const std::ios_base::failure&) { failed = true; }
        CHECK(failed);
        CHECK(drawing.lines().size() == 1);
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
