cmake_minimum_required(VERSION 3.22.1)
project(demo)

set(CMAKE_CXX_STANDARD 17)

add_executable(demo
    main.cpp
    kalman.h
    kalman.cpp
)
