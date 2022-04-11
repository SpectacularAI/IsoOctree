cmake_minimum_required(VERSION 3.3)
project(IsoOctree)

set(CMAKE_CXX_STANDARD 11)

set(ISO_OCTREE_SRC SRC/api.cpp)

add_library(IsoOctree STATIC ${ISO_OCTREE_SRC})
target_include_directories(IsoOctree PRIVATE SRC)
target_compile_options(IsoOctree PRIVATE "-Wno-dangling-else")

add_executable(example SRC/example.cpp)
target_include_directories(example PRIVATE SRC)
target_link_libraries(example PRIVATE IsoOctree)