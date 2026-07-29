cmake_minimum_required(VERSION 3.16)

project(rl_fish_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Stonefish REQUIRED)

add_executable(RLFishSimulator
    src/main.cpp
    src/rl_fish_simulator.cpp
)

target_include_directories(RLFishSimulator
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(RLFishSimulator
    PRIVATE
        Stonefish::Stonefish
)
