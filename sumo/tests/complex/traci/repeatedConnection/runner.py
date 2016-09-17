#!/usr/bin/env python
"""
@file    runner.py
@author  Daniel Krajzewicz
@author  Michael Behrisch
@date    2010-02-20
@version $Id: runner.py 21122 2016-07-06 11:06:52Z behrisch $


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

sumoHome = os.path.abspath(
    os.path.join(os.path.dirname(sys.argv[0]), '..', '..', '..', '..'))
sys.path.append(os.path.join(sumoHome, "tools"))
import sumolib
import traci

PORT = sumolib.miscutils.getFreeSocketPort()
DELTA_T = 1000

if sys.argv[1] == "sumo":
    sumoBinary = os.environ.get(
        "SUMO_BINARY", os.path.join(sumoHome, 'bin', 'sumo'))
    addOption = "--remote-port %s" % PORT
else:
    sumoBinary = os.environ.get(
        "GUISIM_BINARY", os.path.join(sumoHome, 'bin', 'sumo-gui'))
    addOption = "-S -Q --remote-port %s" % PORT


def runSingle(sumoEndTime, traciEndTime):
    fdi = open("sumo.sumocfg")
    fdo = open("used.sumocfg", "w")
    fdo.write(fdi.read() % {"end": sumoEndTime})
    fdi.close()
    fdo.close()
    doClose = True
    step = 0
    sumoProcess = subprocess.Popen(
        "%s -c used.sumocfg %s" % (sumoBinary, addOption), shell=True, stdout=sys.stdout)
    traci.init(PORT)
    while not step > traciEndTime:
        traci.simulationStep()
        vehs = traci.vehicle.getIDList()
        if vehs.index("horiz") < 0 or len(vehs) > 1:
            print("Something is false")
        step += 1
    print("Print ended at step %s" %
          (traci.simulation.getCurrentTime() / DELTA_T))
    traci.close()
    sumoProcess.wait()
    sys.stdout.flush()

print("----------- SUMO ends first -----------")
sys.stdout.flush()
for i in range(0, 10):
    print(" Run %s" % i)
    runSingle(50, 99)

print("----------- TraCI ends first -----------")
sys.stdout.flush()
for i in range(0, 10):
    print(" Run %s" % i)
    runSingle(101, 99)
