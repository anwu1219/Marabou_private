#! /usr/bin/env bash

version=$1
benchmark=$2
onnx=$3
vnnlib=$4

if [[ $onnx == *"gz" ]]
then
    echo "gz file..."
    onnx=${onnx::-3}
    echo $onnx
    if [ -f $onnx ]
    then
        echo "onnx file exists!"
    else
        echo "unzipping..."
        gzip -dk $onnx".gz"
        echo "unzipping - done"
    fi
fi

./../maraboupy/prepare_instance.py $onnx $vnnlib
