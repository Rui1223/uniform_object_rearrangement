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
from UnidirDFSDPPlanner import 
from UnidirCIRSPlanner import UnidirCIRSPlanner

############################### description ###########################################
### This class defines a MonotoneExperimenter class which
### conducts large-scale experiments with different methods
### with the number of experiments specified for each case (#objects)
### It
### (1) asks the execution scene to generate an instance
### (2) asks the pose estimator to get the object poses
### (3) reproduces the instance in task planner 
### (4) solves it with all methods
### (5) collect statistics (e.g., time/actions) from each method for comparison purpose
#######################################################################################

class MonotoneExperimenter(object):

    def __init__(self, args):
        ### set the rospkg path
        rospack = rospkg.RosPack()
        self.rosPackagePath = rospack.get_path("uniform_object_rearrangement")
        self.ExperimentsFolder = os.path.join(self.rosPackagePath, "monotone_experiments")
        if not os.path.exists(self.ExperimentsFolder):
            os.makedirs(self.ExperimentsFolder)
        self.numObjects_options = [7]
        self.numExperiments_perObject = int(args[1])
        self.maxInstancesNeed_perObject = int(args[2])
        self.time_allowed = int(args[3])


    def createNumObjectsFolder(self, num_objects):
        ### create a folder denoted with specified num_objects
        temp_objectFolder = os.path.join(self.ExperimentsFolder, str(num_objects))
        if not os.path.exists(temp_objectFolder):
            os.makedirs(temp_objectFolder)
        temp_nonMonotoneInstancesFolder = \
            os.path.join(self.ExperimentsFolder, str(num_objects), "non_monotone_instances")
        if not os.path.exists(temp_nonMonotoneInstancesFolder):
            os.makedirs(temp_nonMonotoneInstancesFolder)

    def initializeObjLevelStat(self):
        ### initialize obj-level statistics variable
        self.DFS_DP_labeled_success_obj = []
        self.DFS_DP_nonlabeled_success_obj = []
        self.mRS_labeled_success_obj = []
        self.mRS_nonlabeled_success_obj = []
        self.CIRS_success_obj = []
        self.DFS_DP_labeled_time_obj = []
        self.DFS_DP_nonlabeled_time_obj = []
        self.mRS_labeled_time_obj = []
        self.mRS_nonlabeled_time_obj = []
        self.CIRS_time_obj = []


    def saveSolution(self, num_objects, instance_id, solution_time, solution_object_ordering, method_name):
        temp_instanceFolder = os.path.join(self.ExperimentsFolder, str(num_objects), str(instance_id))
        solutionFile = temp_instanceFolder + "/" + method_name + ".txt"
        f_solution = open(solutionFile, "w")
        ################# write in the solution #####################
        f_solution.write(str(solution_time) + "\n")
        for obj_idx in solution_object_ordering:
            f_solution.write(str(obj_idx) + " ")
        f_solution.close()
        #############################################################


    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initializes a ros node
        rospy.init_node("monotone_experiments", anonymous=True)


def main(args):
    monotone_experimenter = MonotoneExperimenter(args)
    monotone_experimenter.rosInit()
    rate = rospy.Rate(10) ### 10hz

    for num_objects in monotone_experimenter.numObjects_options:
        ### create a folder for current num_objects
        monotone_experimenter.createNumObjectsFolder(num_objects)
        monotone_experimenter.initializeObjLevelStat()
        num_monotoneInstancesSaved = 0
        num_nonMonotoneInstancesSaved = 0

        for experiment_id in range(1, monotone_experimenter.numExperiments_perObject+1):
            ### first see if we already have enough instances
            if (num_monotoneInstancesSaved >= monotone_experimenter.maxInstancesNeed_perObject): break
            ### generate an instance in the execution scene
            initialize_instance_success = utils2.serviceCall_generateInstanceCylinder(
                                                num_objects, num_monotoneInstancesSaved+1, True)
            if not initialize_instance_success: continue
            ### object pose estimation
            cylinder_objects = utils2.serviceCall_cylinderPositionEstimate()
            ### reproduce the estimated object poses in the planning scene
            initial_arrangement, final_arrangement, reproduce_instance_success = \
                    utils2.serviceCall_reproduceInstanceCylinder(cylinder_objects)
            ### generate IK config for start positions for all objects
            ik_generate_success = utils2.serviceCall_generateConfigsForStartPositions("Right_torso")

            ########################## now using different methods to solve the instance ##########################
            ### (v) CIRS
            start_time = time.time()
            unidir_cirs_planner = UnidirCIRSPlanner(
                initial_arrangement, final_arrangement, monotone_experimenter.time_allowed)
            cirs_planning_time = time.time() - start_time
            cirs_isSolved = unidir_cirs_planner.isSolved

            if not cirs_isSolved:
                ### the problem is not monotone
                ### add this instance in the "non_monotone" subfolder
                num_nonMonotoneInstancesSaved += 1
                utils2.saveInstance(
                    num_objects, num_nonMonotoneInstancesSaved, cylinder_objects, monotone_experimenter.ExperimentsFolder)

                num_objects, num_monotoneInstancesSaved, cylinder_objects


            ### (i) DFS_DP_labeled
            start_time = time.time()
            unidir_dfsdp_planner = UnidirDFSDPPlanner(initial_arrangement, final_arrangement)
            DFS_DP_labeled_planning_time = time.time() - start_time
            isSolved = unidir_dfsdp_planner.isSolved
            if isSolved:
                DFS_DP_labeled_object_ordering = unidir_dfsdp_planner.object_ordering

            reset_instance_success = monotone_experimenter.resetInstance("Right_torso")

            ### (ii) DFS_DP_nonlabeled
            start_time = time.time()
            unidir_dfsdp_planner = UnidirDFSDPPlanner(initial_arrangement, final_arrangement, isLabeledRoadmapUsed=False)
            DFS_DP_nonlabeled_planning_time = time.time() - start_time
            isSolved = unidir_dfsdp_planner.isSolved
            if isSolved:
                DFS_DP_nonlabeled_object_ordering = unidir_dfsdp_planner.object_ordering

            reset_instance_success = monotone_experimenter.resetInstance("Right_torso")

            ### (iii) mRS_labeled
            start_time = time.time()
            unidir_mrs_planner = UnidirMRSPlanner(initial_arrangement, final_arrangement)
            mRS_labeled_planning_time = time.time() - start_time
            isSolved = unidir_mrs_planner.isSolved
            if isSolved:
                mRS_labeled_object_ordering = unidir_mrs_planner.object_ordering
            
            reset_instance_success = monotone_experimenter.resetInstance("Right_torso")

            ### (iv) mRS_nonlabeled
            start_time = time.time()
            unidir_mrs_planner = UnidirMRSPlanner(initial_arrangement, final_arrangement, isLabeledRoadmapUsed=False)
            mRS_nonlabeled_planning_time = time.time() - start_time
            isSolved = unidir_mrs_planner.isSolved
            if isSolved:
                mRS_nonlabeled_object_ordering = unidir_mrs_planner.object_ordering
            #######################################################################################################

            if isSolved:
                num_monotoneInstancesSaved += 1
                monotone_experimenter.saveInstance(num_objects, num_monotoneInstancesSaved, cylinder_objects)
                monotone_experimenter.saveSolution(
                    num_objects, num_monotoneInstancesSaved, \
                    DFS_DP_labeled_planning_time, DFS_DP_labeled_object_ordering, "official")
                print("\n")
                print("Time for DFS_DP_labeled planning is: {}".format(DFS_DP_labeled_planning_time))
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

            ## Before moving on to the next instance, clear the current instance
            clear_instance_success = monotone_experimenter.clearInstance("Right_torso")
            input("check the instance clearance!!!")


    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)
