cmake_minimum_required(VERSION 3.3)
project(IsoOctree)

set(CMAKE_CXX_STANDARD 11)

set(ISO_OCTREE_SRC SRC/Main.cpp SRC/CmdLineParser.cpp)

add_executable(IsoOctree ${ISO_OCTREE_SRC})
target_include_directories(IsoOctree PRIVATE SRC)
