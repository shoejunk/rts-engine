cmake_minimum_required(VERSION 3.20)

project(rts_engine
    VERSION 0.1.0
    DESCRIPTION "RTS Engine"
    LANGUAGES CXX
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

add_executable(rts_engine
    src/main.cpp
)
target_link_libraries(rts_engine PRIVATE rts_math)
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
