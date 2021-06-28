#! /usr/bin/env bash

version=$1
benchmark=$2
onnx=$3
vnnlib=$4

pkill Marabou

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

benchmark_dir=$(realpath "$SCRIPT_DIR"/benchmarks/)

if [[ ! -d $benchmark_dir ]]
then
    mkdir $benchmark_dir
fi

"$SCRIPT_DIR"/../maraboupy/prepare_instance.py $onnx $vnnlib $benchmark_dir
