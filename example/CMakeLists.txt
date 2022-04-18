cmake_minimum_required(VERSION 3.3)
project(IsoOctreeExample)

add_executable(example main.cpp)
find_package(IsoOctree REQUIRED)
target_link_libraries(example PUBLIC IsoOctree::IsoOctree)
