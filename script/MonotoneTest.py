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
from uniform_object_rearrangement.srv import ExecuteTrajectory, ExecuteTrajectoryRequest
from uniform_object_rearrangement.srv import AttachObject, AttachObjectRequest

from UnidirMRSPlanner import UnidirMRSPlanner

############################### description #########################################
### This class defines a MonotoneTest class which
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
        self.method_name = args[4]

    def saveInstance(self, num_objects, instance_id, cylinder_objects):
        ### create a folder denoted with specified (num_objects, instance_id)
        temp_instanceFolder = os.path.join(self.exampleFolder, str(num_objects), str(instance_id))
        if not os.path.exists(temp_instanceFolder):
            os.makedirs(temp_instanceFolder)
        instanceFile = temp_instanceFolder + "/" + "instance_info.txt"
        f_instance = open(instanceFile, "w")
        ################## write in the instance ####################
        for cylinder_object in cylinder_objects:
            f_instance.write(str(cylinder_object.obj_idx) + "\n")
            f_instance.write(str(cylinder_object.curr_position.x) + " " + \
                str(cylinder_object.curr_position.y) + " " + str(cylinder_object.curr_position.z) + "\n")
        f_instance.close()
        #############################################################

    def saveSolution(self, num_objects, instance_id, solution_time, solution_object_ordering, method_name):
        temp_instanceFolder = os.path.join(self.exampleFolder, str(num_objects), str(instance_id))
        solutionFile = temp_instanceFolder + "/" + method_name + ".txt"
        f_solution = open(solutionFile, "w")
        ################# write in the solution #####################
        f_solution.write(str(solution_time) + "\n")
        for obj_idx in solution_object_ordering:
            f_solution.write(str(obj_idx) + " ")
        f_solution.close()
        #############################################################

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

        ####### now use the specified method to solve the instance #######
        # if monotone_tester.method_name == "DFS_DP_labeled":
        #     start_time = time.time()
        #     TASK_SUCCESS, object_ordering = \
        #         rearrangement_task_planner.DFS_DP_task_planning(len(cylinder_objects))
        #     planning_time = time.time() - start_time
        #     print("Time for DFS_DP_labeled planning is: {}".format(planning_time))

        # if monotone_tester.method_name == "DFS_DP_nonlabeled":
        #     start_time = time.time()
        #     TASK_SUCCESS, object_ordering = \
        #         rearrangement_task_planner.DFS_DP_task_planning(len(cylinder_objects), isLabeledRoadmapUsed=False)
        #     planning_time = time.time() - start_time
        #     print("Time for DFS_DP_nonlabeled planning is: {}".format(planning_time))

        if monotone_tester.method_name == "mRS_labeled":
            start_time = time.time()
            unidir_mrs_planner = UnidirMRSPlanner(initial_arrangement, final_arrangement)
            planning_time = time.time() - start_time
            print("Time for mRS_labeled planning is: {}".format(planning_time))
            isSolved = unidir_mrs_planner.isSolved
            if isSolved:
                object_ordering = unidir_mrs_planner.object_ordering
                object_paths = unidir_mrs_planner.object_paths
                print("object_ordering: {}".format(object_ordering))

        if monotone_tester.method_name == "mRS_nonlabeled":
            start_time = time.time()
            unidir_mrs_planner = UnidirMRSPlanner(initial_arrangement, final_arrangement, isLabeledRoadmapUsed=False)
            planning_time = time.time() - start_time
            print("Time for mRS_nonlabeled planning is: {}".format(planning_time))
            isSolved = unidir_mrs_planner.isSolved
            if isSolved:
                object_ordering = unidir_mrs_planner.object_ordering
                object_paths = unidir_mrs_planner.object_paths
                print("object_ordering: {}".format(object_ordering))

        saveInstanceAndSolution = True if input("save instance & solution?(y/n)") == 'y' else False
        print("save instance and solution: " + str(saveInstanceAndSolution))
        if saveInstanceAndSolution:
            monotone_tester.saveInstance(
                monotone_tester.num_objects, monotone_tester.instance_id, cylinder_objects)
            monotone_tester.saveSolution(
                monotone_tester.num_objects, monotone_tester.instance_id, \
                planning_time, object_ordering, monotone_tester.method_name)
        if isSolved:
            input("enter to start the execution!!!!!")
            execute_success = monotone_tester.executeWholePlan(object_paths)


    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)