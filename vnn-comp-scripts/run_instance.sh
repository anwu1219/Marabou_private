#! /usr/bin/env bash

version=$1
benchmark=$2
onnx=$3
vnnlib=$4
result=$5
timeout=$6

home=$HOME
export INSTALL_DIR="$home"
export GUROBI_HOME="$home/gurobi912/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
export GRB_LICENSE_FILE="$home/gurobi.lic"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

benchmark_dir=$(realpath "$SCRIPT_DIR"/benchmarks/)

case $benchmark in
    cifar2020)
        echo "preparing cifar2020..."
        "$SCRIPT_DIR"/../maraboupy/prepare_instance.py $onnx $vnnlib $benchmark_dir
        echo "preparing - done"
        "$SCRIPT_DIR"/../maraboupy/run_instance.py $onnx $vnnlib $result $benchmark_dir --timeout $6
        ;;
    *)
        "$SCRIPT_DIR"/../maraboupy/run_instance.py $onnx $vnnlib $result $benchmark_dir --timeout $6
        ;;
esac
