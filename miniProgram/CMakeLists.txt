cmake_minimum_required(VERSION 3.12)
project(miniProgram)

set(CMAKE_CXX_STANDARD 14)

add_executable(miniProgram combination/src/main.cpp combination/src/MakeRank.cpp combination/include/MakeRank.h)