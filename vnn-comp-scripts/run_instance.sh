#! /usr/bin/env bash

version=$1
benchmark=$2
onnx=$3
vnnlib=$4
result=$5
timeout=$6

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

benchmark_dir=$(realpath "$SCRIPT_DIR"/benchmarks/)
echo "$SCRIPT_DIR/../maraboupy/run_instance.py $onnx $vnnlib $result $benchmark_dir --timeout $timeout"
"$SCRIPT_DIR"/../maraboupy/run_instance.py $onnx $vnnlib $result $benchmark_dir --timeout $6
