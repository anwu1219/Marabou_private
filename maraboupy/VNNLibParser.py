'''
Top contributors (to current version):
    - Andrew Wu

This file is part of the Marabou project.
Copyright (c) 2017-2019 by the authors listed in the file AUTHORS
in the top-level source directory) and their institutional affiliations.
All rights reserved. See the file COPYING in the top-level source
directory for licensing information.

'''

import os

class VNNLibParser():
    """Parses a vnnlib file and add the constraints to the network object.

    Args:
        filename (str): Path to the vnnlib file
        network (MarabouNetwork.MarabouNetwork: the network object

    """
    def __init__(self, filename, network):
        assert(os.path.isfile(filename))
        self.readVNNLibFile(filename, network)

    def readVNNLibFile(self, filename, network)
