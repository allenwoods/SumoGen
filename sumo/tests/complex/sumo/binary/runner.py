#!/usr/bin/env python
"""
@file    runner.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2010-03-02
@version $Id: runner.py 20433 2016-04-13 08:00:14Z behrisch $


SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import
from __future__ import print_function

import os
import subprocess
import sys
sys.path.append(
    os.path.join(os.path.dirname(sys.argv[0]), '..', '..', '..', '..', "tools"))
import sumolib

netconvertBinary = sumolib.checkBinary('netconvert')
duarouterBinary = sumolib.checkBinary('duarouter')
sumoBinary = sumolib.checkBinary('sumo')

print(">>> Building the xml network")
sys.stdout.flush()
subprocess.call([netconvertBinary, "-c", "netconvert.netccfg"],
                stdout=sys.stdout, stderr=sys.stderr)
print(">>> Building the binary network")
sys.stdout.flush()
subprocess.call([netconvertBinary, "-c", "netconvert.netccfg",
                 "-o", "circular.net.sbx"], stdout=sys.stdout, stderr=sys.stderr)
sys.stdout.flush()
print(">>> Converting the routes to binary")
subprocess.call([duarouterBinary, "-c", "duarouter.duarcfg"],
                stdout=sys.stdout, stderr=sys.stderr)
sys.stdout.flush()
print(">>> Running Simulation with binary input")
subprocess.call(
    [sumoBinary, "-c", "sumo.sumocfg"], stdout=sys.stdout, stderr=sys.stderr)
