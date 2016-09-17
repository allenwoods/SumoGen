"""
@file    costFunction.py
@author  Michael Behrisch
@date    2009-09-08
@version $Id: costFunction.py 20433 2016-04-13 08:00:14Z behrisch $


SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2016 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""


def constant(edge, weight):
    if edge == "middle" or edge == "left":
        return weight
    return 0


def w(edge, weight):
    if edge == "middle":
        return weight + 2
    if edge == "left":
        return weight
    return 1
