#include "editor/scene.h"
#include "editor/canvas_view.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <locale>
#include <stdexcept>
#include <string>

using namespace rts;
using namespace rts::editor;
namespace {
int checks = 0, failures = 0;
#define CHECK(...) do { ++checks; if (!(__VA_ARGS__)) { ++failures; \
    std::printf("FAIL line %d: %s\n", __LINE__, #__VA_ARGS__); } } while (false)
CdtPoint p(int x, int y) { return {Fixed(x), Fixed(y)}; }
bool near(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y) < 0.002f; }
std::string read(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
void wall(Scene& scene, Point a, Point b)
{
    CHECK(scene.begin_line(a, 0));
    CHECK(scene.finish_line(b, 0) == LineDrawing::Error::none);
}
struct CommaDecimal : std::numpunct<char> { char do_decimal_point() const override { return ','; } };
} // namespace

int main()
{
    CanvasView view;
    view.resize(1200, 800);
    const auto anchor = view.to_canvas(670, 390);
    view.zoom_at(3, 670, 390);
    CHECK(view.zoom() > 1);
    CHECK(near(anchor, view.to_canvas(670, 390)));
    CHECK(near(view.to_window(view.to_canvas(240, 350)), {240, 350}));
    view.zoom_at(-3, 670, 390);
    CHECK(std::abs(view.zoom() - 1) < 0.001f);
    const auto beforeToolbar = view.zoom();
    view.zoom_at(3, 670, 50);
    CHECK(view.zoom() == beforeToolbar);
    for (int i = 0; i < 20; ++i) view.zoom_at(10, 670, 390);
    CHECK(view.zoom() == 8);
    for (int i = 0; i < 20; ++i) view.zoom_at(-10, 670, 390);
    CHECK(view.zoom() == 0.25f);
    view.reset(); view.resize(1600, 1000);
    CHECK(view.zoom() == 1);
    CHECK(near(view.to_canvas(view.to_window({500, 400}).x, view.to_window({500, 400}).y), {500, 400}));

    Scene editing;
    CHECK(!editing.place_unit({3, 50}, 12));
    CHECK(!editing.place_unit({50, 50}, 1));
    CHECK(!editing.place_unit({50, 50}, 65));
    CHECK(editing.place_unit({100.25f, 100.5f}, 12));
    CHECK(!editing.place_unit({110, 100}, 12));
    CHECK(editing.place_unit({200, 100}, 24));
    wall(editing, {400, 100}, {400, 600});
    CHECK(!editing.place_unit({405, 300}, 12));
    CHECK(editing.begin_line({50, 100}, 0));
    CHECK(editing.finish_line({300, 100}, 0) == LineDrawing::Error::invalid_vertex);
    CHECK(editing.drawing().lines().size() == 1); // No wall through authored units.
    CHECK(editing.undo());
    CHECK(editing.drawing().lines().empty());
    CHECK(editing.units().size() == 2);
    CHECK(editing.undo());
    CHECK(editing.units().size() == 1);

    Scene scene;
    CHECK(scene.place_unit({100, 100}, 10));
    CHECK(scene.place_unit({150, 100}, 14));
    CHECK(scene.place_unit({800, 600}, 20));
    CHECK(scene.begin_line({300, 200}, 0));
    scene.set_playing(true);
    CHECK(!scene.drawing().preview());
    CHECK(!scene.place_unit({300, 300}, 12));
    CHECK(!scene.begin_line({300, 200}, 0));
    CHECK(!scene.undo());
    scene.finish_line({300, 500}, 0);
    CHECK(scene.drawing().lines().empty());
    scene.select({180, 130}, {70, 70}, false, false); // Reverse drag.
    CHECK(scene.selected_count() == 2);
    CHECK(scene.move_selected({600, 400}) == 2);
    scene.tick();
    CHECK(scene.units()[0].position != p(100, 100));
    for (int i = 0; i < 500; ++i) scene.tick();
    CHECK(scene.units()[0].position == p(575, 400));
    CHECK(scene.units()[1].position == p(625, 400));
    CHECK(scene.units()[2].position == p(800, 600));
    scene.select({0, 0}, {800, 600}, true, true);
    CHECK(scene.selected_count() == 3);
    scene.select({1100, 600}, {1100, 600}, false, true);
    CHECK(scene.selected_count() == 0);
    CHECK(scene.move_selected({400, 400}) == 0);
    scene.set_playing(false);
    CHECK(scene.units()[0].position == p(100, 100));
    scene.tick();
    CHECK(scene.units()[1].position == p(150, 100));
    scene.set_playing(true);
    CHECK(scene.selected_count() == 0);
    scene.select({90, 90}, {115, 115}, false, false);
    CHECK(scene.selected_count() == 1);
    CHECK(scene.move_selected({200, 100}) == 1);
    scene.tick();
    CHECK(scene.move_selected({100, 200}) == 1); // Replace an active move command.
    for (int i = 0; i < 100; ++i) scene.tick();
    CHECK(scene.units()[0].position == p(100, 200));

    Scene detour;
    wall(detour, {600, 100}, {600, 600});
    CHECK(detour.place_unit({300, 350}, 12));
    detour.set_playing(true);
    detour.select({280, 330}, {320, 370}, false, false);
    CHECK(detour.move_selected({900, 350}) == 1);
    CHECK(detour.units()[0].route.size() > 1);
    bool safe = true;
    for (int i = 0; i < 1000; ++i) {
        detour.tick();
        const auto position = display_point(detour.units()[0].position);
        // Independent floating reference for distance to this vertical wall.
        const double distance = std::hypot(position.x - 600.0, position.y - std::clamp<double>(position.y, 100, 600));
        safe = safe && distance >= 11.999;
    }
    CHECK(safe);
    CHECK(detour.units()[0].position == p(900, 350));
    const auto repeat = find_route(p(300, 350), p(900, 350), Fixed(12), constraint_walls(detour.drawing()));
    CHECK(repeat == detour.units()[0].route);

    Scene blocked;
    wall(blocked, {600, 0}, {600, 700});
    CHECK(blocked.place_unit({300, 350}, 12));
    blocked.set_playing(true);
    blocked.select({300, 350}, {300, 350}, false, true);
    CHECK(blocked.move_selected({900, 350}) == 0);
    for (int i = 0; i < 30; ++i) blocked.tick();
    CHECK(blocked.units()[0].position == p(300, 350));
    CHECK(blocked.move_selected({600, 350}) == 0);

    Scene corridor;
    wall(corridor, {200, 330}, {1000, 330});
    wall(corridor, {200, 370}, {1000, 370});
    const auto walls = constraint_walls(corridor.drawing());
    CHECK(!find_route(p(300, 350), p(900, 350), Fixed(12), walls).empty());
    CHECK(find_route(p(300, 350), p(900, 350), Fixed(24), walls).empty());

    const auto folder = std::filesystem::temp_directory_path()
        / ("rts-scene-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(folder);
    const auto first = folder / std::filesystem::path(u8"scene \u00e9.json"), second = folder / "second.json";
    const auto oldLocale = std::locale();
    try {
        std::locale::global(std::locale(oldLocale, new CommaDecimal));
        SceneFile document;
        CHECK(!document.has_path());
        bool failed = false;
        try { document.save(editing); } catch (const std::logic_error&) { failed = true; }
        CHECK(failed);
        document.save_as(editing, first);
        CHECK(document.path() == first);
        const auto original = read(first);
        CHECK(original.find("\"x\": 100.25, \"y\": 100.5, \"radius\": 12") != std::string::npos);
        CHECK(editing.place_unit({200, 200}, 32));
        document.save(editing);
        CHECK(read(first).find("\"radius\": 32") != std::string::npos);
        document.save_as(editing, second);
        CHECK(document.path() == second);
        const auto oldFirst = read(first);
        CHECK(editing.place_unit({300, 200}, 16));
        document.save(editing);
        CHECK(read(first) == oldFirst);
        CHECK(read(second).find("\"x\": 300") != std::string::npos);
        failed = false;
        try { document.save_as(editing, folder / "missing" / "scene.json"); } catch (const std::ios_base::failure&) { failed = true; }
        CHECK(failed);
        CHECK(document.path() == second);
        document.save(scene); // Play movement does not overwrite authored placements.
        CHECK(read(second).find("\"x\": 100, \"y\": 100, \"radius\": 10") != std::string::npos);
        CHECK(read(second).find("selected") == std::string::npos);
        document.save(detour);
        CHECK(read(second).find("\"start\": [600, 100], \"end\": [600, 600]") != std::string::npos);
        CHECK(read(second).find("\"x\": 300, \"y\": 350, \"radius\": 12") != std::string::npos);
    } catch (const std::exception& error) {
        CHECK(false);
        std::printf("Unexpected exception: %s\n", error.what());
    }
    std::locale::global(oldLocale);
    std::filesystem::remove(first); std::filesystem::remove(second); std::filesystem::remove(folder);
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
