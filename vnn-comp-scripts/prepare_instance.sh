#! /usr/bin/env bash

version=$1
benchmark=$2
onnx=$3
vnnlib=$4

pkill Marabou
pkill python

home=$HOME
export INSTALL_DIR="$home"
export GUROBI_HOME="$home/gurobi912/linux64"
export PATH="${PATH}:${GUROBI_HOME}/bin"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${GUROBI_HOME}/lib"
export GRB_LICENSE_FILE="$home/gurobi.lic"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

benchmark_dir=$(realpath "$SCRIPT_DIR"/benchmarks/)

if [[ ! -d $benchmark_dir ]]
then
    mkdir $benchmark_dir
fi

case $benchmark in
    acasxu)
        "$SCRIPT_DIR"/../maraboupy/prepare_instance.py $onnx $vnnlib $benchmark_dir
        ;;
    cifar10_resnet)
        python3 $eran_file  --netname $ONNX_FILE --vnnlib_spec $VNNLIB_FILE --domain refinegpupoly --prepare
        ;;
    cifar2020)
        python3 $eran_file  --netname $ONNX_FILE --vnnlib_spec $VNNLIB_FILE --domain refinegpupoly --prepare
        ;;
    eran)
        if [[ "$ONNX_FILE" == *"SIGMOID"* ]]; then
            python3 $eran_file  --netname $ONNX_FILE --vnnlib_spec $VNNLIB_FILE --domain refinepoly --prepare
        else
            python3 -m onnxsim $onnx "$onnx"-simp
            mv "$onnx"-simp $onnx
            "$SCRIPT_DIR"/../maraboupy/prepare_instance.py $onnx $vnnlib $benchmark_dir
        fi
        ;;
    marabou-cifar10)
        "$SCRIPT_DIR"/../maraboupy/prepare_instance.py $onnx $vnnlib $benchmark_dir
        ;;
    mnistfc)
        "$SCRIPT_DIR"/../maraboupy/prepare_instance.py $onnx $vnnlib $benchmark_dir
        ;;
    nn4sys)
        exit 1
        ;;
    oval21)
        "$SCRIPT_DIR"/../maraboupy/prepare_instance.py $onnx $vnnlib $benchmark_dir
        ;;
    verivital)
        exit 1
        ;;
    *)
        exit 1
        ;;
esac
