cmake_minimum_required(VERSION 3.12)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../VERSION.txt" VERSION_CONTENTS)
project(IsoOctree VERSION ${VERSION_CONTENTS})

option(IsoOctree_PYTHON "Build python bindings" OFF)
set(IsoOctree_PYTHON ON)

add_subdirectory(pybind11)
add_subdirectory(.. ${CMAKE_BINARY_DIR}/IsoOctreeNative_build)

pybind11_add_module(IsoOctree bindings.cpp)
target_include_directories(IsoOctree PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_link_libraries(IsoOctree PRIVATE IsoOctreeNative)