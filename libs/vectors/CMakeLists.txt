cmake_minimum_required(VERSION 2.8.3)
project(vectors_project)

include_directories(
    ../../include
)

add_library(vectors STATIC
    vectors.cpp
)