cmake_minimum_required(VERSION 3.24)

project(rts_engine
    VERSION 0.1.0
    DESCRIPTION "RTS Engine"
    LANGUAGES C CXX
)

# Header-only deterministic math layer. Everything in the simulation depends on
# this and nothing else.
set(RTS_MATH_HEADERS
    src/math/fixed.h
    src/math/angle.h
)

add_library(rts_math INTERFACE)
target_sources(rts_math INTERFACE
    FILE_SET HEADERS
    BASE_DIRS src
    FILES ${RTS_MATH_HEADERS}
)
target_include_directories(rts_math INTERFACE src)
target_compile_features(rts_math INTERFACE cxx_std_20)

# An INTERFACE library generates no project of its own, so without this the
# headers would be invisible in Solution Explorer (and in Xcode, Qt Creator and
# friends). This utility target exists purely to surface them in the IDE: it is
# not part of ALL_BUILD and compiles nothing, so it costs no build time.
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" FILES ${RTS_MATH_HEADERS})
add_custom_target(rts_math_headers SOURCES ${RTS_MATH_HEADERS})

add_library(rts_geometry INTERFACE)
target_link_libraries(rts_geometry INTERFACE rts_math)
target_sources(rts_geometry INTERFACE
    FILE_SET HEADERS BASE_DIRS src FILES src/geometry/cdt.h
)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" FILES src/geometry/cdt.h)
add_custom_target(rts_geometry_headers SOURCES src/geometry/cdt.h)

# Pin and verify the graphics dependency. Static linking keeps the run scripts
# usable without copying an SDL DLL beside the executable.
include(FetchContent)
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_TESTS OFF CACHE BOOL "" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(SDL3
    URL https://github.com/libsdl-org/SDL/releases/download/release-3.4.16/SDL3-3.4.16.tar.gz
    URL_HASH SHA256=7322236cd12090c3eb40b9728be4d49c76f66ad17d04369584d4ecad5cf77c68
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    TIMEOUT 120
    INACTIVITY_TIMEOUT 30
)
FetchContent_MakeAvailable(SDL3)

add_library(rts_line_drawing src/editor/line_drawing.cpp src/editor/line_drawing.h)
target_include_directories(rts_line_drawing PUBLIC src)
target_compile_features(rts_line_drawing PUBLIC cxx_std_20)

add_executable(rts_engine WIN32
    src/main.cpp
)
target_link_libraries(rts_engine PRIVATE rts_math rts_line_drawing SDL3::SDL3-static)
target_compile_features(rts_engine PRIVATE cxx_std_20)

if(MSVC)
    target_compile_options(rts_engine PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(rts_engine PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()

add_executable(rts_math_tests
    tests/fixed_tests.cpp
)
target_link_libraries(rts_math_tests PRIVATE rts_math)
target_compile_features(rts_math_tests PRIVATE cxx_std_20)

if(MSVC)
    target_compile_options(rts_math_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(rts_math_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Golden hash of a fixed workload run through every operation. It is identical
# for MSVC Debug and Release; if a future change, compiler or platform moves it,
# replays recorded before the change will desync, and this test says so loudly.
# Baselined for the 1.17.14 format - changing Fixed::kFracBits changes it.
target_compile_definitions(rts_math_tests PRIVATE
    RTS_EXPECTED_FINGERPRINT=0xC4A58C732BE3F827ull
)

add_test(NAME rts_math_tests COMMAND rts_math_tests)

add_executable(rts_cdt_tests tests/cdt_tests.cpp)
target_link_libraries(rts_cdt_tests PRIVATE rts_geometry)
if(MSVC)
    target_compile_options(rts_cdt_tests PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(rts_cdt_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME rts_cdt_tests COMMAND rts_cdt_tests)

add_executable(rts_line_drawing_tests tests/line_drawing_tests.cpp)
target_link_libraries(rts_line_drawing_tests PRIVATE rts_line_drawing)
foreach(target rts_line_drawing rts_line_drawing_tests)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endforeach()
add_test(NAME rts_line_drawing_tests COMMAND rts_line_drawing_tests)
