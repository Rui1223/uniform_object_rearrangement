#!/usr/bin/env python
from __future__ import division

import time
import sys
import os
import numpy as np

import utils2

import rospy
import rospkg

from UnidirMRSPlanner import UnidirMRSPlanner
from UnidirDFSDPPlanner import UnidirDFSDPPlanner
from UnidirCIRSPlanner import UnidirCIRSPlanner

############################### description #########################################
### This class defines a MonotoneTester class which
### solves an rearrangement problem/example with 
### the number of the object specified
### It
### (1) asks the execution scene to generate an instance
### (2) asks the pose estimator to get the object poses
### (3) reproduces the instance in task planner
### (4) solves it with all methods
### (4) compares the solutions of all methods within the same instance
#####################################################################################

class MonotoneTester(object):

    def __init__(self, args):
        ### set the rospkg path
        rospack = rospkg.RosPack()
        self.rosPackagePath = rospack.get_path("uniform_object_rearrangement")
        self.num_objects = int(args[1])
        self.instance_id = int(args[2])
        self.isNewInstance = True if args[3] == 'g' else False
        self.time_allowed = int(args[4])
        self.instanceFolder = os.path.join(
            self.rosPackagePath, "examples", str(self.num_objects), str(self.instance_id))

    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initializes a ros node
        rospy.init_node("monotone_test", anonymous=True)


def main(args):
    monotone_tester = MonotoneTester(args)
    monotone_tester.rosInit()
    rate = rospy.Rate(10) ### 10hz

    ### generate/load an instance in the execution scene
    initialize_instance_success = utils2.serviceCall_generateInstanceCylinder(
        monotone_tester.num_objects, monotone_tester.instance_id, monotone_tester.isNewInstance)
    if initialize_instance_success:
        ### object pose estimation
        cylinder_objects = utils2.serviceCall_cylinderPositionEstimate()
        ### reproduce the estimated object poses in the planning scene
        initial_arrangement, final_arrangement, reproduce_instance_success = \
                utils2.serviceCall_reproduceInstanceCylinder(cylinder_objects)
        ### generate IK config for start positions for all objects
        ik_generate_success = utils2.serviceCall_generateConfigsForStartPositions("Right_torso")
        
        all_methods_time = []
        all_methods_success = [] ### 0: fail, 1: success
        all_methods_nActions = []

        ###### now using different methods to solve the instance ######
        ### (i) CIRS
        start_time = time.time()
        unidir_cirs_planner = UnidirCIRSPlanner(
            initial_arrangement, final_arrangement, monotone_tester.time_allowed)
        cirs_planning_time = time.time() - start_time
        cirs_isSolved = unidir_cirs_planner.isSolved
        cirs_nActions = unidir_cirs_planner.best_solution_cost
        if cirs_nActions == np.inf:
            cirs_nActions = 5000
        cirs_object_ordering = unidir_cirs_planner.object_ordering
        all_methods_time.append(cirs_planning_time)
        all_methods_success.append(float(cirs_isSolved))
        all_methods_nActions.append(cirs_nActions)

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        ### (ii) DFS_DP_labeled
        start_time = time.time()
        unidir_dfsdp_planner = UnidirDFSDPPlanner(
            initial_arrangement, final_arrangement, monotone_tester.time_allowed)
        DFS_DP_labeled_planning_time = time.time() - start_time
        DFS_DP_labeled_isSolved = unidir_dfsdp_planner.isSolved
        DFS_DP_labeled_nActions = unidir_dfsdp_planner.best_solution_cost
        if DFS_DP_labeled_nActions == np.inf:
            DFS_DP_labeled_nActions = 5000
        DFS_DP_labeled_object_ordering = unidir_dfsdp_planner.object_ordering
        all_methods_time.append(DFS_DP_labeled_planning_time)
        all_methods_success.append(float(DFS_DP_labeled_isSolved))
        all_methods_nActions.append(DFS_DP_labeled_nActions)

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        ### (iii) DFS_DP_nonlabeled
        start_time = time.time()
        unidir_dfsdp_planner = UnidirDFSDPPlanner(
            initial_arrangement, final_arrangement, monotone_tester.time_allowed, 
            isLabeledRoadmapUsed=False)
        DFS_DP_nonlabeled_planning_time = time.time() - start_time
        DFS_DP_nonlabeled_isSolved = unidir_dfsdp_planner.isSolved
        DFS_DP_nonlabeled_nActions = unidir_dfsdp_planner.best_solution_cost
        if DFS_DP_nonlabeled_nActions == np.inf:
            DFS_DP_nonlabeled_nActions = 5000
        DFS_DP_nonlabeled_object_ordering = unidir_dfsdp_planner.object_ordering
        all_methods_time.append(DFS_DP_nonlabeled_planning_time)
        all_methods_success.append(float(DFS_DP_nonlabeled_isSolved))
        all_methods_nActions.append(DFS_DP_nonlabeled_nActions)

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        ### (iv) mRS_labeled
        start_time = time.time()
        unidir_mrs_planner = UnidirMRSPlanner(
            initial_arrangement, final_arrangement, monotone_tester.time_allowed)
        mRS_labeled_planning_time = time.time() - start_time
        mRS_labeled_isSolved = unidir_mrs_planner.isSolved
        mRS_labeled_nActions = unidir_mrs_planner.best_solution_cost
        if mRS_labeled_nActions == np.inf:
            mRS_labeled_nActions = 5000
        mRS_labeled_object_ordering = unidir_mrs_planner.object_ordering
        all_methods_time.append(mRS_labeled_planning_time)
        all_methods_success.append(float(mRS_labeled_isSolved))
        all_methods_nActions.append(mRS_labeled_nActions)

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        ### (v) mRS_nonlabeled
        start_time = time.time()
        unidir_mrs_planner = UnidirMRSPlanner(
            initial_arrangement, final_arrangement, monotone_tester.time_allowed, 
            isLabeledRoadmapUsed=False)
        mRS_nonlabeled_planning_time = time.time() - start_time
        mRS_nonlabeled_isSolved = unidir_mrs_planner.isSolved
        mRS_nonlabeled_nActions = unidir_mrs_planner.best_solution_cost
        if mRS_nonlabeled_nActions == np.inf:
            mRS_nonlabeled_nActions = 5000
        mRS_nonlabeled_object_ordering = unidir_mrs_planner.object_ordering
        all_methods_time.append(mRS_nonlabeled_planning_time)
        all_methods_success.append(float(mRS_nonlabeled_isSolved))
        all_methods_nActions.append(mRS_nonlabeled_nActions)

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        print("\n")
        print("Time for CIRS planning is: {}".format(cirs_planning_time))
        print("Success for CIRS planning is: {}".format(cirs_isSolved))
        print("Number of actions for CIRS planning: {}".format(cirs_nActions))
        print("Object ordering for CIRS planning is: {}".format(cirs_object_ordering))
        print("\n")
        print("Time for DFS_DP_labeled planning is: {}".format(DFS_DP_labeled_planning_time))
        print("Success for DFS_DP_labeled_planning is: {}".format(DFS_DP_labeled_isSolved))
        print("Number of actions for DFS_DP_labeled planning is: {}".format(DFS_DP_labeled_nActions))
        print("Object ordering for DFS_DP_labeled planning is: {}".format(DFS_DP_labeled_object_ordering))
        print("\n")
        print("Time for DFS_DP_nonlabeled planning is: {}".format(DFS_DP_nonlabeled_planning_time))
        print("Success for DFS_DP_nonlabeled planning is: {}".format(DFS_DP_nonlabeled_isSolved))
        print("Number of actions for DFS_DP_nonlabeled planning is: {}".format(DFS_DP_nonlabeled_nActions))
        print("Object ordering for DFS_DP_nonlabeled planning is: {}".format(DFS_DP_nonlabeled_object_ordering))
        print("\n")
        print("Time for mRS_labeled planning is: {}".format(mRS_labeled_planning_time))
        print("Success for mRS_labeled planning is: {}".format(mRS_labeled_isSolved))
        print("Number of actions for mRS_labeled planning is: {}".format(mRS_labeled_nActions))
        print("Object ordering for mRS_labeled planning is: {}".format(mRS_labeled_object_ordering))
        print("\n")
        print("Time for mRS_nonlabeled planning is: {}".format(mRS_nonlabeled_planning_time))
        print("Success for mRS_nonlabeled planning is: {}".format(mRS_nonlabeled_isSolved))
        print("Number of actions for mRS_nonlabeled planning is: {}".format(mRS_nonlabeled_nActions))
        print("Object ordering for mRS_nonlabeled planning is: {}".format(mRS_nonlabeled_object_ordering))
        print("\n")

        if monotone_tester.isNewInstance:
            ### only keep the option to save instance when it is a new instance
            saveInstance = True if input("save instance? (y/n)") == 'y' else False
            print("save instance: " + str(saveInstance))
            if saveInstance:
                utils2.saveInstance(
                    monotone_tester.num_objects, monotone_tester.instance_id, 
                    cylinder_objects, monotone_tester.exampleFolder)
        saveSolution = True if input("save solution? (y/n)") == 'y' else False
        print("save solution: " + str(saveSolution))
        if saveSolution:
            utils2.saveSolution(all_methods_time, all_methods_success, all_methods_nActions,
                                monotone_tester.instanceFolder)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)