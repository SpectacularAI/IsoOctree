#!/bin/bash
set -eux

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
