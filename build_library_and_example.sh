#!/bin/bash
set -eux

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

rm -rf target
mkdir -p target
cd target
cmake -DCMAKE_INSTALL_PREFIX=/tmp/IsoOctreeInstall -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j8 install
cd ..

cd example
rm -rf target
mkdir -p target
cd target
cmake -DCMAKE_PREFIX_PATH=/tmp/IsoOctreeInstall ..
make -j8

./example test.obj
wc test.obj
