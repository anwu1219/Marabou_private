#! /usr/bin/env bash

onnx=$1
vnnlib=$2
result=$3
timeout=$4

./prepare_instance.sh v1 doncare $onnx $vnnlib
./run_instance.sh v1 doncare $onnx $vnnlib $result $timeout
