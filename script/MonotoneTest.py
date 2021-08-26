#!/usr/bin/env python
from __future__ import division

import time
import sys
import os
import numpy as np

import rospy
import rospkg

from uniform_object_rearrangement.msg import CylinderObj
from uniform_object_rearrangement.srv import GenerateInstanceCylinder, GenerateInstanceCylinderRequest
from uniform_object_rearrangement.srv import CylinderPositionEstimate, CylinderPositionEstimateRequest
from uniform_object_rearrangement.srv import ReproduceInstanceCylinder, ReproduceInstanceCylinderRequest
from uniform_object_rearrangement.srv import GenerateConfigsForStartPositions, GenerateConfigsForStartPositionsRequest
from uniform_object_rearrangement.srv import ResetPlanningInstance, ResetPlanningInstanceRequest
from uniform_object_rearrangement.srv import ClearPlanningInstance, ClearPlanningInstanceRequest
from uniform_object_rearrangement.srv import ClearExecutionInstance, ClearExecutionInstanceRequest
from uniform_object_rearrangement.srv import ResetRoadmap, ResetRoadmapRequest
from uniform_object_rearrangement.srv import ExecuteTrajectory, ExecuteTrajectoryRequest
from uniform_object_rearrangement.srv import AttachObject, AttachObjectRequest

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
### (3) reproduces the instance in task planner and solve it with a specified planner
### (4) asks the planning scene to plan the manipulation paths
### (5) asks the execution scene to execute the planned paths
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

    def saveInstance(self, num_objects, instance_id, cylinder_objects):
        ### create a folder denoted with specified (num_objects, instance_id)
        temp_instanceFolder = os.path.join(self.exampleFolder, str(num_objects), str(instance_id))
        if not os.path.exists(temp_instanceFolder):
            os.makedirs(temp_instanceFolder)
        ################## write in the instance ####################
        instanceFile = temp_instanceFolder + "/" + "instance_info.txt"
        f_instance = open(instanceFile, "w")
        for cylinder_object in cylinder_objects:
            f_instance.write(str(cylinder_object.obj_idx) + "\n")
            f_instance.write(str(cylinder_object.curr_position.x) + " " + \
                str(cylinder_object.curr_position.y) + " " + str(cylinder_object.curr_position.z) + "\n")
        f_instance.close()
        #############################################################

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

    def serviceCall_generateInstanceCylinder(self, num_objects, instance_number, isNewInstance):
        rospy.wait_for_service("generate_instance_cylinder")
        request = GenerateInstanceCylinderRequest()
        request.num_objects = num_objects
        request.instance_number = instance_number
        request.isNewInstance = isNewInstance
        try:
            generateInstanceCylinder_proxy = rospy.ServiceProxy(
                        "generate_instance_cylinder", GenerateInstanceCylinder)
            generate_instance_cylinder_response = generateInstanceCylinder_proxy(
                    request.num_objects, request.instance_number, request.isNewInstance)
            return generate_instance_cylinder_response.success
        except rospy.ServiceException as e:
            print("generate_instance_cylinder service call failed: %s" % e)

    def serviceCall_cylinderPositionEstimate(self):
        rospy.wait_for_service("cylinder_position_estimate")
        request = CylinderPositionEstimateRequest()
        try:
            cylinderPositionEstimate_proxy = rospy.ServiceProxy(
                "cylinder_position_estimate", CylinderPositionEstimate)
            cylinder_position_estimate_response = cylinderPositionEstimate_proxy(request)
            return cylinder_position_estimate_response.cylinder_objects
        except rospy.ServiceException as e:
            print("cylinder_position_estimate service call failed: %s" % e)

    def serviceCall_reproduceInstanceCylinder(self, cylinder_objects):
        ### Input: cylinder_objects (CylinderObj[])
        rospy.wait_for_service("reproduce_instance_cylinder")
        request = ReproduceInstanceCylinderRequest(cylinder_objects)
        try:
            reproduceInstanceCylinder_proxy = rospy.ServiceProxy(
                "reproduce_instance_cylinder", ReproduceInstanceCylinder)
            reproduce_instance_cylinder_response = reproduceInstanceCylinder_proxy(request.cylinder_objects)
            return list(reproduce_instance_cylinder_response.initial_arrangement), \
                   list(reproduce_instance_cylinder_response.final_arrangement), \
                   reproduce_instance_cylinder_response.success
        except rospy.ServiceException as e:
            print("reproduce_instance_cylinder service call failed: %s" % e)
    
    def serviceCall_generateConfigsForStartPositions(self, armType):
        rospy.wait_for_service("generate_configs_for_start_positions")
        request = GenerateConfigsForStartPositionsRequest()
        request.armType = armType
        try:
            generateConfigsForStartPositions_proxy = rospy.ServiceProxy(
                "generate_configs_for_start_positions", GenerateConfigsForStartPositions)
            generate_configs_for_start_positions_response = generateConfigsForStartPositions_proxy(request.armType)
            return generate_configs_for_start_positions_response.success
        except rospy.ServiceException as e:
            print("generate_configs_for_start_positions service call failed" % e)

    def serviceCall_reset_planning_instance(self):
        rospy.wait_for_service("reset_planning_instance")
        request = ResetPlanningInstanceRequest()
        try:
            resetPlanningInstance_proxy = rospy.ServiceProxy("reset_planning_instance", ResetPlanningInstance)
            reset_planning_instance_response = resetPlanningInstance_proxy(request)
            return reset_planning_instance_response.success
        except rospy.ServiceException as e:
            print("reset_planning_instance service call failed: %s" % e)

    def serviceCall_clear_planning_instance(self):
        rospy.wait_for_service("clear_planning_instance")
        request = ClearPlanningInstanceRequest()
        try:
            clearPlanningInstance_proxy = rospy.ServiceProxy("clear_planning_instance", ClearPlanningInstance)
            clear_planning_instance_response = clearPlanningInstance_proxy(request)
            return clear_planning_instance_response.success
        except rospy.ServiceException as e:
            print("clear_planning_instance service call failed: %s" % e)

    def serviceCall_clear_execution_instance(self):
        rospy.wait_for_service("clear_execution_instance")
        request = ClearExecutionInstanceRequest()
        try:
            clearExecutionInstance_proxy = rospy.ServiceProxy("clear_execution_instance", ClearExecutionInstance)
            clear_execution_instance_response = clearExecutionInstance_proxy(request)
            return clear_execution_instance_response.success
        except rospy.ServiceException as e:
            print("clear_execution_instance service call failed: %s" % e)

    def serviceCall_reset_roadmap(self, armType):
        rospy.wait_for_service("reset_roadmap")
        request = ResetRoadmapRequest()
        request.armType = armType
        try:
            resetRoadmap_proxy = rospy.ServiceProxy("reset_roadmap", ResetRoadmap)
            reset_roadmap_response = resetRoadmap_proxy(request)
            return reset_roadmap_response.success
        except rospy.ServiceException as e:
            print("reset_roadmap service call failed: %s" % e)

    def serviceCall_execute_trajectory(self, traj):
        '''call the ExecuteTrajectory service to execute the given trajectory
        inputs
        ======
            traj (an ArmTrajectory object): the trajectory to execute
        outputs
        =======
            success: indicator of whether the trajectory is executed successfully
        '''
        rospy.wait_for_service("execute_trajectory")
        request = ExecuteTrajectoryRequest()
        request.arm_trajectory = traj
        try:
            executeTraj_proxy = rospy.ServiceProxy("execute_trajectory", ExecuteTrajectory)
            executeTraj_response = executeTraj_proxy(request.arm_trajectory)
            return executeTraj_response.success
        except rospy.ServiceException as e:
            print("execute_trajectory service call failed: %s" % e)

    def serviceCall_attach_object(self, attach, object_idx, armType):
        """call the AttachObject service to attach/detach the corresponding object
        inputs
        ======
            attach (bool): indicate the action of attach or detach
            object_idx (int): the object to attach or detach
            armType (string): "Left"/"Right"/"Left_torso"/"Right_torso"
        outputs
        =======
            success: indicator of whether the attach/detach command is fulfilled
        """
        rospy.wait_for_service("attach_object")
        request = AttachObjectRequest()
        request.attach = attach
        request.object_idx = object_idx
        request.armType = armType
        try:
            attachObject_proxy = rospy.ServiceProxy("attach_object", AttachObject)
            attachObject_response = attachObject_proxy(request.attach, request.object_idx, request.armType)
            return attachObject_response.success
        except rospy.ServiceException as e:
            print(" attach_object service call failed: %s" % e)

    def executeWholePlan(self, whole_path):
        """ call the ExecuteTrajectory service to execute each trajectory in the path
            also tell the robot to attach or detach the object among the motions 
            inputs
            ======
                whole path (a list of ObjectRearrangementPath): a sequence of object paths
            outputs
            =======
                execute_success (bool): indicate whether success or not
        """
        for path in whole_path:
            ### first execute the transit trajectory in the path
            execute_success = self.serviceCall_execute_trajectory(path.transit_trajectory)
            ### now attach the object
            attach_success = self.serviceCall_attach_object(
                attach=True, object_idx=path.object_idx, armType=path.transit_trajectory.armType)
            ### then execute the transfer trajectory in the path
            execute_success = self.serviceCall_execute_trajectory(path.transfer_trajectory)
            ### now detach the object
            attach_success = self.serviceCall_attach_object(
                attach=False, object_idx=path.object_idx, armType=path.transit_trajectory.armType)
            ### finally execute the finish trajectory in the path
            execute_success = self.serviceCall_execute_trajectory(path.finish_trajectory)

        return execute_success

    def resetInstance(self, armType):
        reset_planning_success = self.serviceCall_reset_planning_instance()
        reset_roadmap_success = self.serviceCall_reset_roadmap(armType)
        if reset_planning_success and reset_roadmap_success:
            return True
        else:
            return False

    def clearInstance(self, armType):
        clear_planning_success = self.serviceCall_clear_planning_instance()
        clear_execution_success = self.serviceCall_clear_execution_instance()
        reset_roadmap_success = self.serviceCall_reset_roadmap(armType)
        if clear_planning_success and clear_execution_success and reset_roadmap_success:
            return True
        else:
            return False


    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initializes a ros node
        rospy.init_node("monotone_test", anonymous=True)


def main(args):
    monotone_tester = MonotoneTester(args)
    monotone_tester.rosInit()
    rate = rospy.Rate(10) ### 10hz

    ### generate/load an instance in the execution scene
    initialize_instance_success = monotone_tester.serviceCall_generateInstanceCylinder(
        monotone_tester.num_objects, monotone_tester.instance_id, monotone_tester.isNewInstance)
    if initialize_instance_success:
        ### object pose estimation
        cylinder_objects = monotone_tester.serviceCall_cylinderPositionEstimate()
        ### reproduce the estimated object poses in the planning scene
        initial_arrangement, final_arrangement, reproduce_instance_success = \
                monotone_tester.serviceCall_reproduceInstanceCylinder(cylinder_objects)
        ### generate IK config for start positions for all objects
        ik_generate_success = monotone_tester.serviceCall_generateConfigsForStartPositions("Right_torso")
        
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
        reset_instance_success = monotone_tester.resetInstance("Right_torso")
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
        reset_instance_success = monotone_tester.resetInstance("Right_torso")
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
        reset_instance_success = monotone_tester.resetInstance("Right_torso")
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
        reset_instance_success = monotone_tester.resetInstance("Right_torso")
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
                monotone_tester.saveInstance(
                    monotone_tester.num_objects, monotone_tester.instance_id, cylinder_objects)
        saveSolution = True if input("save solution? (y/n)") == 'y' else False
        print("save solution: " + str(saveSolution))
        if saveSolution:
            monotone_tester.saveSolution(
                monotone_tester.num_objects, monotone_tester.instance_id, all_methods_time, all_methods_success)


    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)