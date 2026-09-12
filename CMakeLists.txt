cmake_minimum_required(VERSION 3.20)

project(rts_engine
    VERSION 0.1.0
    DESCRIPTION "RTS Engine"
    LANGUAGES CXX
)

add_executable(rts_engine
    src/main.cpp
)

target_compile_features(rts_engine PRIVATE cxx_std_20)

if(MSVC)
    target_compile_options(rts_engine PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(rts_engine PRIVATE -Wall -Wextra -Wpedantic)
endif()
