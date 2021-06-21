#! /usr/bin/env python3
'''
Top contributors (to current version):
    - Andrew Wu

This file is part of the Marabou project.
Copyright (c) 2017-2021 by the authors listed in the file AUTHORS
in the top-level source directory) and their institutional affiliations.
All rights reserved. See the file COPYING in the top-level source
directory for licensing information.
'''

import argparse
import numpy as np
import os
import sys
import tempfile
import pathlib
sys.path.insert(0, os.path.join(str(pathlib.Path(__file__).parent.absolute()), "../"))
from maraboupy import Marabou
from maraboupy import MarabouCore
from maraboupy import MarabouUtils
import subprocess

def main():
        args = arguments().parse_args()
        query, filename = createQuery(args)
        print("Writing query file to {}".format(filename))
        if query == None:
                print("Unable to create an input query!")
                print("There are three options to define the benchmark:\n"
                  "1. Provide an input query file.\n"
                  "2. Provide a network and a property file.\n"
                  "3. Provide a network, a dataset (--dataset), an epsilon (-e), "
                  "target label (-t), and the index of the point in the test set (-i).")
                exit(1)
        MarabouCore.saveQuery(query, filename)

def getInputQueryName(networkPath, propPath, workDir):
    netname = os.path.basename(networkPath)
    propname = os.path.basename(propPath)
    return os.path.join(workDir, netname + "-" + propname)

def createQuery(args):
    networkPath = args.network
    propPath = args.prop
    suffix = networkPath.split('.')[-1]
    assert(suffix == "onnx")
    network = Marabou.read_onnx(networkPath)
    if args.debug:
        inputs = np.random.uniform(0,1, np.array(network.inputVars).shape)
        outputOnnx = network.evaluateWithoutMarabou(inputs)
        outputMarabou = network.evaluateWithMarabou(inputs)
        assert((abs(outputOnnx - outputMarabou) < 0.00001).all())
        print("output onnx:", outputOnnx)
        print("output marabou:", outputMarabou)

    return network.getMarabouQuery(), getInputQueryName(networkPath, propPath, args.work_dir)

def arguments():
    ################################ Arguments parsing ##############################
    parser = argparse.ArgumentParser(description="Script to run some canonical benchmarks with Marabou (e.g., ACAS benchmarks, l-inf robustness checks on mnist/cifar10).")
    # benchmark
    parser.add_argument('network', type=str, nargs='?', default=None,
                        help='The network file name, the extension can be only .pb, .nnet, and .onnx')
    parser.add_argument('prop', type=str, nargs='?', default=None,
                        help='The property file name')
    parser.add_argument('--work-dir', type=str, default="./",
                        help='The working dir')
    parser.add_argument('--debug', action="store_true", default=False)

    return parser

if __name__ == "__main__":
    main()
