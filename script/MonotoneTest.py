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
        self.exampleFolder = os.path.join(self.rosPackagePath, "examples")
        if not os.path.exists(self.exampleFolder):
            os.makedirs(self.exampleFolder)
        self.num_objects = int(args[1])
        self.instance_id = int(args[2])
        self.isNewInstance = True if args[3] == 'g' else False
        self.time_allowed = int(args[4])


    def saveSolution(self, num_objects, instance_id, all_methods_time, all_methods_success):
        temp_instanceFolder = os.path.join(self.exampleFolder, str(num_objects), str(instance_id))
        timeFile = temp_instanceFolder + "/time.txt"
        f_time = open(timeFile, "w")
        for method_time in all_methods_time:
            f_time.write(str(method_time) + "\n")
        f_time.close()
        successFile = temp_instanceFolder + "/success.txt"
        f_success = open(successFile, "w")
        for method_success in all_methods_success:
            f_success.write(str(int(method_success)) + "\n")
        f_success.close()

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
        ###### now using different methods to solve the instance ######
        ### (i) DFS_DP_labeled
        start_time = time.time()
        unidir_dfsdp_planner = UnidirDFSDPPlanner(
            initial_arrangement, final_arrangement, monotone_tester.time_allowed)
        DFS_DP_labeled_planning_time = time.time() - start_time
        all_methods_time.append(DFS_DP_labeled_planning_time)
        DFS_DP_labeled_isSolved = unidir_dfsdp_planner.isSolved
        all_methods_success.append(DFS_DP_labeled_isSolved)
        if DFS_DP_labeled_isSolved:
            DFS_DP_labeled_object_ordering = unidir_dfsdp_planner.object_ordering
            DFS_DP_labeled_object_paths = unidir_dfsdp_planner.object_paths
        else:
            DFS_DP_labeled_object_ordering = []
            DFS_DP_labeled_object_paths = []

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        ### (ii) DFS_DP_nonlabeled
        start_time = time.time()
        unidir_dfsdp_planner = UnidirDFSDPPlanner(
            initial_arrangement, final_arrangement, monotone_tester.time_allowed, isLabeledRoadmapUsed=False)
        DFS_DP_nonlabeled_planning_time = time.time() - start_time
        all_methods_time.append(DFS_DP_nonlabeled_planning_time)
        DFS_DP_nonlabeled_isSolved = unidir_dfsdp_planner.isSolved
        all_methods_success.append(DFS_DP_nonlabeled_isSolved)
        if DFS_DP_nonlabeled_isSolved:
            DFS_DP_nonlabeled_object_ordering = unidir_dfsdp_planner.object_ordering
            DFS_DP_nonlabeled_object_paths = unidir_dfsdp_planner.object_paths
        else:
            DFS_DP_nonlabeled_object_ordering = []
            DFS_DP_nonlabeled_object_paths = []

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        ### (iii) mRS_labeled
        start_time = time.time()
        unidir_mrs_planner = UnidirMRSPlanner(initial_arrangement, final_arrangement, monotone_tester.time_allowed)
        mRS_labeled_planning_time = time.time() - start_time
        all_methods_time.append(mRS_labeled_planning_time)
        mRS_labeled_isSolved = unidir_mrs_planner.isSolved
        all_methods_success.append(mRS_labeled_isSolved)
        if mRS_labeled_isSolved:
            mRS_labeled_object_ordering = unidir_mrs_planner.object_ordering
            mRS_labeled_object_paths = unidir_mrs_planner.object_paths
        else:
            mRS_labeled_object_ordering = []
            mRS_labeled_object_paths = []

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        ### (iv) mRS_nonlabeled
        start_time = time.time()
        unidir_mrs_planner = UnidirMRSPlanner(
            initial_arrangement, final_arrangement, monotone_tester.time_allowed, isLabeledRoadmapUsed=False)
        mRS_nonlabeled_planning_time = time.time() - start_time
        all_methods_time.append(mRS_nonlabeled_planning_time)
        mRS_nonlabeled_isSolved = unidir_mrs_planner.isSolved
        all_methods_success.append(mRS_nonlabeled_isSolved)
        if mRS_nonlabeled_isSolved:
            mRS_nonlabeled_object_ordering = unidir_mrs_planner.object_ordering
            mRS_nonlabeled_object_paths = unidir_mrs_planner.object_paths
        else:
            mRS_nonlabeled_object_ordering = []
            mRS_nonlabeled_object_paths = []

        #####################################################################
        reset_instance_success = utils2.resetInstance("Right_torso")
        #####################################################################

        ### (v) CIRS
        start_time = time.time()
        unidir_cirs_planner = UnidirCIRSPlanner(initial_arrangement, final_arrangement, monotone_tester.time_allowed)
        cirs_planning_time = time.time() - start_time
        all_methods_time.append(cirs_planning_time)
        cirs_isSolved = unidir_cirs_planner.isSolved
        all_methods_success.append(cirs_isSolved)
        if cirs_isSolved:
            cirs_object_ordering = unidir_cirs_planner.object_ordering
            cirs_object_paths = unidir_cirs_planner.object_paths
        else:
            cirs_object_ordering = []
            cirs_object_paths = []

        print("\n")
        print("Time for DFS_DP labeled planning is: {}".format(DFS_DP_labeled_planning_time))
        print("Object ordering for DFS_DP_labeled planning is: {}".format(DFS_DP_labeled_object_ordering))
        print("\n")
        print("Time for DFS_DP_nonlabeled planning is: {}".format(DFS_DP_nonlabeled_planning_time))
        print("Object ordering for DFS_DP_nonlabeled planning is: {}".format(DFS_DP_nonlabeled_object_ordering))
        print("\n")
        print("Time for mRS_labeled planning is: {}".format(mRS_labeled_planning_time))
        print("Object ordering for mRS_labeled planning is: {}".format(mRS_labeled_object_ordering))
        print("\n")
        print("Time for mRS_nonlabeled planning is: {}".format(mRS_nonlabeled_planning_time))
        print("Object ordering for mRS_nonlabeled planning is: {}".format(mRS_nonlabeled_object_ordering))
        print("\n")
        print("Time for CIRS planning is: {}".format(cirs_planning_time))
        print("Object ordering for CIRS planning is: {}".format(cirs_object_ordering))
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
            monotone_tester.saveSolution(
                monotone_tester.num_objects, monotone_tester.instance_id, all_methods_time, all_methods_success)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)