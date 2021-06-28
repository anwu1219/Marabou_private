#! /usr/bin/env bash

version=$1
benchmark=$2
onnx=$3
vnnlib=$4
result=$5
timeout=$6

benchmark_dir=$(realpath ./benchmarks/)
./../maraboupy/run_instance.py $onnx $vnnlib $result $benchmark_dir --timeout $6
