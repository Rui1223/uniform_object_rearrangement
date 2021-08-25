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

from uniform_object_rearrangement.srv import DetectInvalidArrStates, DetectInvalidArrStatesRequest


# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__


class CIRSSolver(MonotoneLocalSolver):
    def __init__(self, startArrNode, target_arrangement, isLabeledRoadmapUsed=True):
        MonotoneLocalSolver.__init__(
            self, startArrNode, target_arrangement, isLabeledRoadmapUsed)
        rospy.logwarn("a CIRSSolver starts to work")
        self.explored = [] ### a list of arrangements which have been explored
        ### we also need a dict to store all invalid arrangements 
        ### at which each object is manipulated (self.invalid_arr_per_obj)
        ### format: {obj_idx: [{invalid_arr1}, {invalid_arr2}, {invalid_arr3}], ...}
        self.invalid_arr_per_obj = {}
    
    def cirs_solve(self):
        ### before the search, given start_arrangement and target_arrangement
        ### detect all invalid arrangement at which each object to be manipulated
        self.detectInvalidArrStates()
        LOCAL_TASK_SUCCESS = self.CIDFS_DP()
        return LOCAL_TASK_SUCCESS, self.tree

    def detectInvalidArrStates(self):
        '''This function detects all invalid states of arrangement
        at which each object is manipulated in the local task'''
        invalid_states = self.serviceCall_detectInvalidArrStates()

    def CIDFS_DP(self):
        '''search towards final arrangement based on current arrangement'''
        ###### return FLAG==true if the problem is solved by CIDFS_DP (an indication of monotonicity) ######
        #### TO BE COMPLETED...
        ### THIS FUNCTION IS ALMOST THE SAME AS DFS_DP WITH ONE EXCEPTION
        ### ONLINE CHECKING IF THE ARRANGEMENT IS IN THE INVALID STATES OF ARRANGEMENT
        pass


    def serviceCall_detectInvalidArrStates(self):
        rospy.wait_for_service("detect_invalid_arr_states")
        request = DetectInvalidArrStatesRequest()
        request.start_arrangement = self.start_arrangement
        request.target_arrangement = self.target_arrangement
        try:
            detectInvalidArrStates_proxy = rospy.ServiceProxy(
                "detect_invalid_arr_states", DetectInvalidArrStates)
            detect_invalid_arr_states_response = detectInvalidArrStates_proxy(request)
            return detect_invalid_arr_states_response.success
        except rospy.ServiceException as e:
            print("detect_invalid_arr_states service call failed: %s" % e)
    

