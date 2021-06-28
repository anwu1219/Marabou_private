#! /usr/bin/env bash

apt-get update -y
apt-get install sudo -y
sudo apt-get update -y
sudo apt-get install python3 -y
sudo apt-get install python3-pip -y

project_path=$(dirname "$script_path")
cd $project_path

# install marabou
rm -rf build
mkdir build
cd build
cmake ./ -DENABLE_GUROBI=ON
make -j48
cd ../
