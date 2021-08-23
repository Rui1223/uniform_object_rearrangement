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
from MRSSolver import MRSSolver


# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__


class UnidirMRSPlanner(RearrangementTaskPlanner):
    def __init__(self, initial_arrangement, final_arrangement, isLabeledRoadmapUsed=True):
        RearrangementTaskPlanner.__init__(
            self, initial_arrangement, final_arrangement, isLabeledRoadmapUsed)
        rospy.logwarn("initialize an unidirectional MRS planner")

        self.growSubTree(self.treeL["L0"], self.final_arrangement, self.isLabeledRoadmapUsed)
        if self.isSolved:
            self.harvestSolution()

    def growSubTree(self, rootNode, target_arrangement, isLabeledRoadmapUsed):
        ### (i) generate the subTree
        mrs_solver = MRSSolver(
            rootNode, target_arrangement, isLabeledRoadmapUsed)
        local_task_success, subTree = mrs_solver.mrs_solve()
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
        idToID = OrderedDict()
        queue = [0]
        idToID[0] = rootNode.node_id

        while (len(queue) != 0):
            parent_id = queue.pop()
            parent_nodeID = idToID[parent_id]
            parent_arrangement = self.treeL[parent_nodeID].arrangement
            ### get all the children of this parent node
            if parent_id not in child_dict.keys():
                ### no children, this is leaf node
                children_ids = []
            else:
                children_ids = child_dict[parent_id]
            for child_id in children_ids:
                ### first check if this child arrangement has already been in the tree
                isSameNodeInTheTree, same_nodeID = self.checkSameArrangementNodeInTheLeftTree(subTree[child_id])
                if isSameNodeInTheTree:
                    idToID[child_id] = same_nodeID
                    ### we have this node in the tree before,
                    ### Since the ordering is the same, no need to rewire
                else:
                    ### this is a new node to be added to the search tree
                    self.left_idx += 1
                    self.treeL["L"+str(self.left_idx)] = copy.deepcopy(subTree[child_id]) 
                    self.treeL["L"+str(self.left_idx)].updateNodeID("L"+str(self.left_idx))
                    self.treeL["L"+str(self.left_idx)].updateParent(parent_nodeID)
                    self.arrLeftRegistr.append(subTree[child_id].arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
                    self.orderLeftRegistr.append(subTree[child_id].object_ordering)
                    idToID[child_id] = "L"+str(self.left_idx)
                    ### check if we reach the FINAL_ARRANGEMENT
                    if subTree[child_id].arrangement == self.final_arrangement:
                        rospy.logwarn("SOLUTION HAS BEEN FOUND")
                        self.isSolved = True
                        self.finalNodeID = "L"+str(self.left_idx)
                        return
                
                ### before move on to other children, add this child into the queue for future expansion
                queue.insert(0, child_id)
            
            ### reach here as all the children have been explored. Move on to next parent in the queue

    
    def checkSameArrangementNodeInTheLeftTree(self, arr_node):
        '''This function checks if an arrangement node is already in the search tree (left)
        It returns (1) same or not (bool) (2) if same, the node ID (string)'''
        object_ordering = arr_node.object_ordering
        if object_ordering not in self.orderLeftRegistr:
            return False, None
        else:
            similar_arrangement_nodeID = self.idLeftRegistr[self.orderLeftRegistr.index(object_ordering)]
            return True, similar_arrangement_nodeID