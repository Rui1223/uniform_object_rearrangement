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
from uniform_object_rearrangement.srv import ClearPlanningInstance, ClearPlanningInstanceRequest
from uniform_object_rearrangement.srv import ClearExecutionInstance, ClearExecutionInstanceRequest
from uniform_object_rearrangement.srv import ResetRoadmap, ResetRoadmapRequest
from uniform_object_rearrangement.srv import ExecuteTrajectory, ExecuteTrajectoryRequest
from uniform_object_rearrangement.srv import AttachObject, AttachObjectRequest

from RearrangementTaskPlanner import RearrangementTaskPlanner

class MonotoneExperimenter(object):

    def __init__(self, args):
        ### set the rospkg path
        rospack = rospkg.RosPack()
        self.rosPackagePath = rospack.get_path("uniform_object_rearrangement")
        self.ExperimentsFolder = os.path.join(self.rosPackagePath, "monotone_experiments")
        if not os.path.exists(self.ExperimentsFolder):
            os.makedirs(self.ExperimentsFolder)
        self.numObjects_options = [6, 8]
        self.numExperiments_perObject = int(args[1])
        self.maxInstancesNeed_perObject = int(args[2])

    def createNumObjectsFolder(self, num_objects):
        ### create a folder denoted with specified num_objects
        temp_objectFolder = os.path.join(self.ExperimentsFolder, str(num_objects))
        if not os.path.exists(temp_objectFolder):
            os.makedirs(temp_objectFolder)

    def saveInstance(self, num_objects, instance_id, cylinder_objects):
        ### create a folder denoted with specified (num_objects, instance_id)
        temp_instanceFolder = os.path.join(self.ExperimentsFolder, str(num_objects), str(instance_id))
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
        temp_instanceFolder = os.path.join(self.ExperimentsFolder, str(num_objects), str(instance_id))
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
            return reproduce_instance_cylinder_response.success
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

    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initializes a ros node
        rospy.init_node("monotone_experiments", anonymous=True)


def main(args):
    monotone_experimenter = MonotoneExperimenter(args)
    rearrangement_task_planner = RearrangementTaskPlanner()
    monotone_experimenter.rosInit()
    rate = rospy.Rate(10) ### 10hz

    for num_objects in monotone_experimenter.numObjects_options:
        ### create a folder for current num_objects
        monotone_experimenter.createNumObjectsFolder(num_objects)
        num_monotoneInstancesSaved = 0
        for experiment_id in range(1, monotone_experimenter.numExperiments_perObject+1):
            ### first see if we already have enough instances
            if (num_monotoneInstancesSaved >= monotone_experimenter.maxInstancesNeed_perObject): break
            ### generate an instance in the execution scene
            initialize_instance_success = monotone_experimenter.serviceCall_generateInstanceCylinder(
                                                                num_objects, num_monotoneInstancesSaved+1, True)
            if not initialize_instance_success: continue
            ### object pose estimation
            cylinder_objects = monotone_experimenter.serviceCall_cylinderPositionEstimate()
            ### reproduce the estimated object poses in the planning scene
            reproduce_instance_success = monotone_experimenter.serviceCall_reproduceInstanceCylinder(cylinder_objects)
            ### generate IK config for start positions for all objects
            ik_generate_success = monotone_experimenter.serviceCall_generateConfigsForStartPositions("Right_torso")

            ########## now using different methods in the RearrangementTaskPlanner to solve the instance ##########
            ## (i) DFS_DP_labeled
            start_time = time.time()
            TASK_SUCCESS, DFS_DP_labeled_object_ordering = \
                        rearrangement_task_planner.DFS_DP_task_planning(len(cylinder_objects))
            DFS_DP_labeled_planning_time = time.time() - start_time
            print("Time for DFS_DP_labeled planning is: {}".format(DFS_DP_labeled_planning_time))

            ### (ii) DFS_DP
            # start_time = time.time()
            # TASK_SUCCESS, DFS_DP_object_ordering = \
            #     rearrangement_task_planner.DFS_DP_task_planning(len(cylinder_objects), isLabeledRoadmapUsed=False)
            # DFS_DP_planning_time = time.time() - start_time
            # print("Time for DFS_DP planning is: {}".format(DFS_DP_planning_time))

            ### (iii) mRS_labeled
            # start_time = time.time()
            # TASK_SUCCESS, mRS_labeled_object_ordering = \
            #             rearrangement_task_planner.mRS_task_planning(len(cylinder_objects))
            # mRS_labeled_planning_time = time.time() - start_time
            # print("Time for mRS_labeled planning is: {}".format(mRS_labeled_planning_time))

            ### (iv) mRS
            # start_time = time.time()
            # TASK_SUCCESS, mRS_object_ordering = \
            #     rearrangement_task_planner.mRS_task_planning(len(cylinder_objects), isLabeledRoadmapUsed=False)
            # mRS_planning_time = time.time() - start_time
            # print("Time for mRS planning is: {}".format(mRS_planning_time))
            #######################################################################################################

            if TASK_SUCCESS:
                num_monotoneInstancesSaved += 1
                monotone_experimenter.saveInstance(num_objects, num_monotoneInstancesSaved, cylinder_objects)
                monotone_experimenter.saveSolution(
                    num_objects, num_monotoneInstancesSaved, \
                    DFS_DP_labeled_planning_time, DFS_DP_labeled_object_ordering, "official")
            
            ## Before moving on to the next instance, clear the current instance
            clear_planning_success = monotone_experimenter.serviceCall_clear_planning_instance()
            clear_execution_success = monotone_experimenter.serviceCall_clear_execution_instance()
            reset_roadmap_success = monotone_experimenter.serviceCall_reset_roadmap("Right_torso")
            input("check the clearance!!!")

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)
