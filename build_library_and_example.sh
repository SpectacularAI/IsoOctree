#!/bin/bash
set -eux

mkdir -p target
cd target
cmake -DCMAKE_INSTALL_PREFIX=/tmp/IsoOctreeInstall -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j8 install
cd ..

cd example
mkdir -p target
cd target
cmake -DCMAKE_PREFIX_PATH=/tmp/IsoOctreeInstall ..
make -j8

./example test.obj
wc test.obj
