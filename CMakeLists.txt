cmake_minimum_required(VERSION 3.25)
project(scooterouter VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SDL2 CONFIG REQUIRED)
find_package(SDL2_ttf CONFIG REQUIRED)
find_package(CLI11 CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(benchmark CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

file(GLOB_RECURSE CORE_SOURCES    src/core/*.cpp)
file(GLOB_RECURSE PLANNER_SOURCES src/planner/*.cpp)
file(GLOB_RECURSE ROUTING_SOURCES src/routing/*.cpp)
file(GLOB_RECURSE MAP_SOURCES     src/map/*.cpp)
file(GLOB_RECURSE VIZ_SOURCES     src/viz/*.cpp)

add_executable(scooterouter
    src/main.cpp
    ${CORE_SOURCES}
    ${PLANNER_SOURCES}
    ${ROUTING_SOURCES}
    ${MAP_SOURCES}
    ${VIZ_SOURCES}
)

target_include_directories(scooterouter PRIVATE include)

target_link_libraries(scooterouter PRIVATE
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
    $<IF:$<TARGET_EXISTS:SDL2_ttf::SDL2_ttf>,SDL2_ttf::SDL2_ttf,SDL2_ttf::SDL2_ttf-static>
    CLI11::CLI11
    fmt::fmt
    nlohmann_json::nlohmann_json
)

add_executable(scooterouter_bench
    benchmark/bench_planners.cpp
    ${CORE_SOURCES}
    ${PLANNER_SOURCES}
    ${ROUTING_SOURCES}
    ${MAP_SOURCES}
)

target_include_directories(scooterouter_bench PRIVATE include)
target_link_libraries(scooterouter_bench PRIVATE
    expat::expat
    benchmark::benchmark
    benchmark::benchmark_main
    fmt::fmt
    nlohmann_json::nlohmann_json
)

file(GLOB TEST_SOURCES tests/*.cpp)

add_executable(scooterouter_tests
    ${TEST_SOURCES}
    ${CORE_SOURCES}
    ${PLANNER_SOURCES}
    ${ROUTING_SOURCES}
    ${MAP_SOURCES}
)

target_include_directories(scooterouter_tests PRIVATE include)
target_link_libraries(scooterouter_tests PRIVATE
    GTest::gtest
    GTest::gtest_main
    fmt::fmt
    nlohmann_json::nlohmann_json
)

enable_testing()
add_test(NAME scooterouter_tests COMMAND scooterouter_tests)

# libosmium requires expat for XML parsing
find_package(expat CONFIG REQUIRED)
target_link_libraries(scooterouter PRIVATE expat::expat)
target_link_libraries(scooterouter_tests PRIVATE expat::expat)
