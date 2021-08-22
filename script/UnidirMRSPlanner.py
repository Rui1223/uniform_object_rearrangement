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

from RearrangementTaskPlanner import RearrangementTaskPlanner
from RearrangementTaskPlanner import ArrNode
from MonotoneLocalSolver import MRSSolver

class UnidirMRSPlanner(RearrangementTaskPlanner):
    def __init__(self, initial_arrangement, final_arrangement, isLabeledRoadmapUsed=True):
        RearrangementTaskPlanner.__init__(
            self, initial_arrangement, final_arrangement, isLabeledRoadmapUsed)
        rospy.logwarn("initialize an unidirectional MRS planner")

        self.growSubTree(self.treeL["L0"], self.final_arrangement, self.isLabeledRoadmapUsed)

    def growSubTree(self, rootNode, target_arrangement, isLabeledRoadmapUsed):
        ### (i) generate the subTree
        local_task_success, subTree = MRSSolver(
            rootNode, target_arrangement, isLabeledRoadmapUsed)
        ### (ii) engraft the subTree to the global search tree
        self.engraftingLeftTree(rootNode, subTree)
        
    def engraftingLeftTree(self, rootNode, subTree):
        if len(subTree) == 1: 
            ### the subTree only contains the rootNode
            ### basically indicates the tree is not growing
            ### then there is nothing to engraft
            return

        ### first construct a child dict, 
        ### so we can use BFS to traverse the tree during engrafting
        child_dict = {} ### key: parent, value: children (list)
        for node_id, arr_node in subTree.items():
            child_id = node_id
            parent_id = arr_node.parent_id
            if parent_id == None:
                continue
            if parent_id not in child_dict.keys():
                child_dict[parent_id] = []
            child_dict[parent_id].append(child_id)

        ### use BFS to add the subTree to the entire global tree structure
        queue = [0]
        parent_arrangement = rootNode.arrangement
        parent_nodeID = rootNode.node_id
        while (len(queue) != 0):
            parent_id = queue.pop()
            ### get all the children of this parent node
            if parent_id not in child_dict.keys():
                ### no children, this is leaf node
                children_ids = []
            else:
                children_ids = child_dict[parent_id]
            for child_id in children_ids:
                child_arrangement = subTree[child_id].arrangement
                ### first check if this child arrangement has already in the tree
                similar_arrangement_
                if child_arrangement in self.arrLeftRegistr:
                     


        


        