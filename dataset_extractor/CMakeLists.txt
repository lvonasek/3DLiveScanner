cmake_minimum_required(VERSION 3.0.2)

find_library(turbojpeg-lib turbojpeg)
find_library(png-lib png)
find_package(OpenCV REQUIRED)

include_directories(
    ../common
    ../third_party/tango_3d_reconstruction/include
)

add_executable(
    dataset_extractor
    ../common/data/dataset.cc
    ../common/data/depthmap.cc
    ../common/data/file3d.cc
    ../common/data/image.cc
    ../common/data/mesh.cc
    ../common/editor/rasterizer.cc
    ../common/exporter/csvposes.cc
    ../common/exporter/depthmaps.cc
    ../common/exporter/exporter.cc
    ../common/exporter/floorpln.cc
    ../common/exporter/ply.cc
    ../common/gl/camera.cc
    ../common/postproc/texturize.cc
    app.cpp
)

target_link_libraries(
    dataset_extractor
    png
    turbojpeg
    ${OpenCV_LIBS}
)
