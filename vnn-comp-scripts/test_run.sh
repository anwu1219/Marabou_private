#! /usr/bin/env bash

family=$1
onnx=$2
vnnlib=$3
result=$4
timeout=$5

./prepare_instance.sh v1 $family $onnx $vnnlib
./run_instance.sh v1 doncare $onnx $vnnlib $result $timeout
