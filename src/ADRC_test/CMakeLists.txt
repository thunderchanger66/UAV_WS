cmake_minimum_required(VERSION 3.8)
project(ADRC_test)

find_package(pybind11 REQUIRED)

pybind11_add_module(ladrc
    src/ladrc.cpp
    src/bindings.cpp)

target_include_directories(ladrc
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
