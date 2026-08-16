cmake_minimum_required(VERSION 3.16)

project(
    RLFishSimulator
    VERSION 0.2.0
    LANGUAGES CXX
)


set(
    CMAKE_CXX_STANDARD
    17
)

set(
    CMAKE_CXX_STANDARD_REQUIRED
    ON
)

set(
    CMAKE_CXX_EXTENSIONS
    OFF
)


set(
    CMAKE_EXPORT_COMPILE_COMMANDS
    ON
)


find_package(
    Stonefish
    REQUIRED
)


# ================================================================
# Existing river simulator
# ================================================================

add_executable(
    RLFishSimulator

    src/main.cpp
    src/rl_fish_simulator.cpp
    src/river_chunk_manager.cpp
)


target_include_directories(
    RLFishSimulator

    PRIVATE

        ${CMAKE_CURRENT_SOURCE_DIR}/src
)


target_link_libraries(
    RLFishSimulator

    PRIVATE

        Stonefish::Stonefish
)


target_compile_options(
    RLFishSimulator

    PRIVATE

        -Wall
        -Wextra
        -Wpedantic
)


# ================================================================
# Bionic Fish V1 - Static Pool Test Bench
# ================================================================

add_executable(
    BionicFishPoolTest

    src/pool_main.cpp
    src/static_pool_simulator.cpp
    src/bionic_fish.cpp
)


target_include_directories(
    BionicFishPoolTest

    PRIVATE

        ${CMAKE_CURRENT_SOURCE_DIR}/src
)


target_link_libraries(
    BionicFishPoolTest

    PRIVATE

        Stonefish::Stonefish
)


target_compile_options(
    BionicFishPoolTest

    PRIVATE

        -Wall
        -Wextra
        -Wpedantic
)
