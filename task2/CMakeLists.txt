cmake_minimum_required(VERSION 3.10)
project(score_app)

include_directories(include)

add_executable(score_app main.cpp src/score_utils.cpp)

