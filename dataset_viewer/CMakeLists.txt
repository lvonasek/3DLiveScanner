cmake_minimum_required(VERSION 3.0.2)

find_library(glut-lib glut)
find_library(GL-lib GL)
find_library(turbojpeg-lib turbojpeg)
find_library(png-lib png)
find_library(X11-lib X11)

include_directories(
    ../common
    ../third_party/tango_3d_reconstruction/include
)

add_executable(
    dataset_viewer
    ../common/data/dataset.cc
    ../common/data/file3d.cc
    ../common/data/image.cc
    ../common/data/mesh.cc
    ../common/gl/glsl.cc
    app.cpp
)

target_link_libraries(
    dataset_viewer
    glut
    GL
    png
    turbojpeg
    X11
)
