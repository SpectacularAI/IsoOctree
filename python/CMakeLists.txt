# Copyright 2024 Spectacular AI Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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