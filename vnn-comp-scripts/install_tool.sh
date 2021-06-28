#! /usr/bin/env bash

sudo apt install gzip

# install marabou
rm -rf build
mkdir build
cd build
cmake ./ -DENABLE_GUROBI=ON
make -j32
cd ../
