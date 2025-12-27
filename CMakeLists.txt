# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
project(WallVisualizer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wall -Wextra")

add_executable(wall_visualizer
    main.cpp
    wall_visualizer.cpp
)