#!/usr/bin/env python
from __future__ import division

import time
import sys
import os
import copy
import numpy as np
from collections import OrderedDict

import rospy
import rospkg

from MonotoneLocalSolver import MonotoneLocalSolver
from MonotoneLocalSolver import ArrNode

class DFSDPSolver(MonotoneLocalSolver):
    def __init__(self, startArrNode, target_arrangement, isLabeledRoadmapUsed=True):
        MonotoneLocalSolver.__init__(
            self, startArrNode, target_arrangement, isLabeledRoadmapUsed)
        rospy.logwarn("a DFSDPSolver start to work")
        ### a list of set() - current object set (e.g, (1,2,3) == (3,2,1))
        self.explored = []