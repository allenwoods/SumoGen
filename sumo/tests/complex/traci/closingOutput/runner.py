#!/usr/bin/env python
"""
@file    runner.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2011-07-01
@version $Id: runner.py 21131 2016-07-08 07:59:22Z behrisch $


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
import time
import shutil

sumoHome = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.append(os.path.join(sumoHome, "tools"))
import sumolib
import traci

PORT = sumolib.miscutils.getFreeSocketPort()

if sys.argv[1] == "sumo":
    sumoBinary = os.environ.get(
        "SUMO_BINARY", os.path.join(sumoHome, 'bin', 'sumo'))
    addOption = []
else:
    sumoBinary = os.environ.get(
        "GUISIM_BINARY", os.path.join(sumoHome, 'bin', 'sumo-gui'))
    addOption = ["-S", "-Q"]

sumoProcess = subprocess.Popen(
    [sumoBinary, "-c", "sumo.sumocfg", "--remote-port", str(PORT)] + addOption)
traci.init(PORT)
time.sleep(10)
step = 0
while not step > 100:
    traci.simulationStep()
    vehs = traci.vehicle.getIDList()
    if vehs.index("horiz") < 0 or len(vehs) > 1:
        print("Something is false")
    step += 1
traci.close()
sumoProcess.wait()
sys.stdout.flush()
