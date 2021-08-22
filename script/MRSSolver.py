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

class MRSSolver(MonotoneLocalSolver):
    def __init__(self, startArrNode, target_arrangement, isLabeledRoadmapUsed=True):
        MonotoneLocalSolver.__init__(
            self, startArrNode, target_arrangement, isLabeledRoadmapUsed)
        rospy.logwarn("a MRSSolver start to work")

        LOCAL_TASK_SUCCESS = self.DFS()
        return LOCAL_TASK_SUCCESS, self.tree


    def DFS(self):
        '''search towards final arrangement based on current arrangement'''
        ###### return FLAG==true if the problem is solved by mRS (an indication of monotonicity) ######
        current_node_id = copy.deepcopy(self.node_idx)
        current_arrangement = self.tree[current_node_id].arrangement

        ### first check if we touch the base case: we are at the target_arrangement
        if (current_arrangement == self.target_arrangement):
            ### the problem is solved
            return True
        ### otherwise it's not solved yet. Check if time exceeds
        if time.time() - self.local_planning_startTime >= self.time_threshold:
            return False

        FLAG = False
        remaining_objects = [i for i in range(len(current_arrangement)) \
                            if current_arrangement[i] != self.target_arrangement[i]] 
        for obj_idx in remaining_objects:
            ### let's check this object by rearranging it
            rearrange_success, transition_path = self.serviceCall_rearrangeCylinderObject(
                    obj_idx, "Right_torso", isLabeledRoadmapUsed=self.isLabeledRoadmapUsed)
            if rearrange_success:
                self.generateLocalNode(current_node_id, obj_idx, transition_path)
                ### recursive call
                FLAG = self.DFS()
                if FLAG: 
                    return FLAG
                else:
                    ### put the object and robot back to the configuration they belong to
                    ### at the beginning of the function call
                    self.revertBackToParentNode(current_node_id, obj_idx, "Right_torso")
            else:
                ### put the object and robot back to the configuration they belong to
                ### at the beginning of the function call
                self.revertBackToParentNode(current_node_id, obj_idx, "Right_torso")

        ### the problem is not solved but there is no option
        return FLAG

    def generateLocalNode(self, current_node_id, obj_idx, transition_path):
        '''generate a local node which has parent node id == current_node_id,
           given the obj_idx and the transition_path'''
        current_arrangement = self.tree[current_node_id].arrangement

        resulting_arrangement = copy.deepcopy(current_arrangement)
        resulting_arrangement[obj_idx] = self.target_arrangement[obj_idx]
        resulting_robot_config = self.serviceCall_getCurrRobotConfig()
        resulting_transit_from_info = [
            self.tree[current_node_id].objectTransferred_idx, \
            self.tree[current_node_id].obj_transfer_position_indices[1]]
        resulting_obj_transfer_position_indices = [current_arrangement[obj_idx], self.target_arrangement[obj_idx]]
        resulting_cost_to_come = self.tree[current_node_id].cost_to_come + 1
        ### add this newly-generated node
        self.node_idx += 1
        self.tree[self.current_node_id] = ArrNode(
            resulting_arrangement, resulting_robot_config, self.node_idx, 
            resulting_transit_from_info, resulting_obj_transfer_position_indices, obj_idx, 
            transition_path, resulting_cost_to_come, current_node_id
        )

    def revertBackToParentNode(self, current_node_id, obj_idx, armType):
        '''revert back to the parent node (pop out operation in DFS) by
        put the object and robot back to the configuration they belong to
        at the beginning of the function call'''
        current_object_position_idx = self.tree[current_node_id].obj_transfer_position_indices[0]
        update_success = self.serviceCall_updateCertainObjectPose(obj_idx, current_object_position_idx)
        current_robot_config = self.tree[current_node_id].robotConfig
        update_success = self.serviceCall_resetRobotCurrConfig(current_robot_config)
        update_success = self.serviceCall_updateManipulationStatus(armType)