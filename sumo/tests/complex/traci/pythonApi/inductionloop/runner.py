#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file    runner.py
@author  Michael Behrisch
@author  Daniel Krajzewicz
@date    2011-03-04
@version $Id: runner.py 21122 2016-07-06 11:06:52Z behrisch $


SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

from __future__ import print_function
from __future__ import absolute_import
import os
import subprocess
import sys
import random
sys.path.append(os.path.join(os.environ['SUMO_HOME'], "tools"))
sys.path.append(os.path.join(
    os.path.dirname(sys.argv[0]), "..", "..", "..", "..", "..", "tools"))
import traci
import sumolib

sumoBinary = sumolib.checkBinary('sumo')

PORT = sumolib.miscutils.getFreeSocketPort()
sumoProcess = subprocess.Popen(
    "%s -c sumo.sumocfg --remote-port %s" % (sumoBinary, PORT), shell=True, stdout=sys.stdout)
traci.init(PORT)
for step in range(4):
    print("step", step)
    traci.simulationStep()
print("inductionloops", traci.inductionloop.getIDList())
print("inductionloop count", traci.inductionloop.getIDCount())
loopID = "0"
print("examining", loopID)
print("vehNum", traci.inductionloop.getLastStepVehicleNumber(loopID))
print("meanSpeed", traci.inductionloop.getLastStepMeanSpeed(loopID))
print("vehIDs", traci.inductionloop.getLastStepVehicleIDs(loopID))
print("occupancy", traci.inductionloop.getLastStepOccupancy(loopID))
print("meanLength", traci.inductionloop.getLastStepMeanLength(loopID))
print("timeSinceDet", traci.inductionloop.getTimeSinceDetection(loopID))
print("vehData", traci.inductionloop.getVehicleData(loopID))

traci.inductionloop.subscribe(loopID)
print(traci.inductionloop.getSubscriptionResults(loopID))
for step in range(3, 6):
    print("step", step)
    traci.simulationStep()
    print(traci.inductionloop.getSubscriptionResults(loopID))

for i in range(24):
    print("step=%s detVehs=%s vehData=%s" % (
        traci.simulation.getCurrentTime() / 1000.0,
        traci.inductionloop.getLastStepVehicleIDs(loopID),
        traci.inductionloop.getVehicleData(loopID),
    ))
    traci.simulationStep()

traci.close()
sumoProcess.wait()
