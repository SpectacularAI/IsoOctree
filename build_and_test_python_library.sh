#!/bin/bash
set -eux

source venv/bin/activate
rm -rf dist
rm -rf python/target

mkdir -p python/target
cd python/target
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
cd ../..

python python/setup.py bdist_wheel
deactivate

cd python/target
python -m venv venv_test
source venv_test/bin/activate
pip install ../../dist/*.whl
cd ..
python example.py