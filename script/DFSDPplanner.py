#!/usr/bin/env python
from __future__ import division

import time
import sys
import os
import numpy as np

import rospy
import rospkg

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from uniform_object_rearrangement.msg import CylinderObj
from uniform_object_rearrangement.msg import ObjectRearrangePath
from uniform_object_rearrangement.srv import GenerateInstanceCylinder, GenerateInstanceCylinderRequest
from uniform_object_rearrangement.srv import CylinderPositionEstimate, CylinderPositionEstimateRequest
from uniform_object_rearrangement.srv import ReproduceInstanceCylinder, ReproduceInstanceCylinderRequest
from uniform_object_rearrangement.srv import RearrangeCylinderObject, RearrangeCylinderObjectRequest
from uniform_object_rearrangement.srv import ExecuteTrajectory, ExecuteTrajectoryRequest
from uniform_object_rearrangement.srv import AttachObject, AttachObjectRequest
from uniform_object_rearrangement.srv import GetCertainObjectPose, GetCertainObjectPoseRequest
from uniform_object_rearrangement.srv import GetCurrRobotConfig, GetCurrRobotConfigRequest
from uniform_object_rearrangement.srv import UpdateCertainObjectPose, UpdateCertainObjectPoseRequest
from uniform_object_rearrangement.srv import ResetRobotCurrConfig, ResetRobotCurrConfigRequest
from uniform_object_rearrangement.srv import UpdateManipulationStatus, UpdateManipulationStatusRequest

############################### description ################################
### This class defines a DFSDPplanner class which
### uses DFS_DP(Dynamic Programming) as the task planner
### to solve the object rearrangement problem
### It
### (1) asks the execution scene to generate an instance
### (2) asks the pose estimator to get the object poses
### (3) reproduces the instance in task planner and solve it with DFSDP planner
### (4) asks the planning scene to plan the manipulation paths
### (5) asks the execution scene to execute the planned paths
#############################################################################


class DFSDPplanner(object):

    def __init__(self, args):
        ### set the rospkg path
        rospack = rospkg.RosPack()
        self.rosPackagePath = rospack.get_path("uniform_object_rearrangement")
        self.num_objects = int(args[1])
        self.all_objects = [i for i in range(self.num_objects)]
        self.object_ordering = [] ### a list of obj_idx
        self.object_paths = [] ### a list of ObjectRearrangePath paths
        self.explored = [] ### a list of set() - current object set (e.g, (1,2,3) == (3,2,1))

    def serviceCall_generateInstanceCylinder(self):
        rospy.wait_for_service("generate_instance_cylinder")
        request = GenerateInstanceCylinderRequest()
        request.num_objects = self.num_objects
        try:
            generateInstanceCylinder_proxy = rospy.ServiceProxy(
                        "generate_instance_cylinder", GenerateInstanceCylinder)
            success = generateInstanceCylinder_proxy(request.num_objects)
            return success.success
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
            success = reproduceInstanceCylinder_proxy(request.cylinder_objects)
            return success.success
        except rospy.ServiceException as e:
            print("reproduce_instance_cylinder service call failed: %s" % e)

    def serviceCall_rearrangeCylinderObject(self, obj_idx, armType):
        rospy.wait_for_service("rearrange_cylinder_object")
        request = RearrangeCylinderObjectRequest()
        request.object_idx = obj_idx
        request.armType = armType
        try:
            rearrangeCylinderObject_proxy = rospy.ServiceProxy(
                "rearrange_cylinder_object", RearrangeCylinderObject)
            rearrange_cylinder_object_response = rearrangeCylinderObject_proxy(request)
            return rearrange_cylinder_object_response.success, rearrange_cylinder_object_response.path
        except rospy.ServiceException as e:
            print("rearrange_cylinder_object service call failed: %s" % e)

    def serviceCall_getCertainObjectPose(self, obj_idx):
        '''call the GetCertainObjectPose service to get the object pose from planning '''
        rospy.wait_for_service("get_certain_object_pose")
        request = GetCertainObjectPoseRequest()
        request.object_idx = obj_idx
        try:
            getObjectPose_proxy = rospy.ServiceProxy("get_certain_object_pose", GetCertainObjectPose)
            getObjectPose_response = getObjectPose_proxy(request.object_idx)
            object_curr_pos = [getObjectPose_response.curr_position.x, \
                getObjectPose_response.curr_position.y, getObjectPose_response.curr_position.z]
            return object_curr_pos
        except rospy.ServiceException as e:
            print("get_certain_object_pose service call failed: %s" % e)

    def serviceCall_getCurrRobotConfig(self):
        '''call the GetCurrRobotConfig service to get the robot current config from planning
           expect output: configuration of all controllable joints (1 + 7 + 7 + 6) '''
        rospy.wait_for_service("get_curr_robot_config")
        request = GetCurrRobotConfigRequest()
        try:
            getCurrRobotConfig_proxy = rospy.ServiceProxy("get_curr_robot_config", GetCurrRobotConfig)
            getCurrRobotConfig_response = getCurrRobotConfig_proxy(request)
            return getCurrRobotConfig_response.robot_config.position
        except rospy.ServiceException as e:
            print("get_curr_robot_config service call failed: %s" % e)

    def serviceCall_updateCertainObjectPose(self, obj_idx, target_pose):
        '''call the UpdateCertainObjectPose service to update the object
           to the specified target pose'''
        rospy.wait_for_service("update_certain_object_pose")
        request = UpdateCertainObjectPoseRequest()
        request.object_idx = obj_idx
        request.target_pose = Point(target_pose[0], target_pose[1], target_pose[2])
        try:
            updateCertainObjectPose_proxy = rospy.ServiceProxy("update_certain_object_pose", UpdateCertainObjectPose)
            updateCertainObjectPose_response = updateCertainObjectPose_proxy(request.object_idx, request.target_pose)
            return updateCertainObjectPose_response.success
        except rospy.ServiceException as e:
            print("update_certain_object_pose service call failed: %s" % e)

    def serviceCall_resetRobotCurrConfig(self, robot_curr_config):
        '''call the ResetRobotCurrConfig service to reset the robot
           to the specified configuration'''
        rospy.wait_for_service("reset_robot_curr_config")
        request = ResetRobotCurrConfigRequest()
        request.robot_config = JointState()
        request.robot_config.position = robot_curr_config
        try:
            resetRobotCurrConfig_proxy = rospy.ServiceProxy("reset_robot_curr_config", ResetRobotCurrConfig)
            resetRobotCurrConfig_response = resetRobotCurrConfig_proxy(request.robot_config)
            return resetRobotCurrConfig_response.success
        except rospy.ServiceException as e:
            print("reset_robot_curr_config service call failed: %s" % e)
    
    def serviceCall_updateManipulationStatus(self, armType):
        '''call the UpdateManipulationStatus service to disable
           any relationship between the robot and the object'''
        rospy.wait_for_service("update_manipulation_status")
        request = UpdateManipulationStatusRequest()
        request.armType = armType
        try:
            updateManipulationStatus_proxy = rospy.ServiceProxy("update_manipulation_status", UpdateManipulationStatus)
            updateManipulationStatus_response = updateManipulationStatus_proxy(request.armType)
            return updateManipulationStatus_response.success
        except rospy.ServiceException as e:
            print("update_manipulation_status service call failed: %s" % e)


    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initializes a ros node
        rospy.init_node("dfsdp_task_planner", anonymous=True)

    def DFS_DP(self):
        '''search for the remaining object ordering based on current object ordering
           but it remembers all the explored status'''
        ### (1) update self.object_ordering and self.paths (a list of ObjectRearrangePath paths)
        ### (2) increment the explored list (item: set())
        ### (3) return FLAG==true if the problem is solved by DFS_DP (an indication of monotonicity)   
        print("object_ordering: " + str(self.object_ordering))
        FLAG = False
        ### check the base case: object_ordering has been fully filled
        if (len(self.object_ordering) == self.num_objects):
            return True
        for obj_idx in self.all_objects:
            if obj_idx in self.object_ordering:
                continue
            if set(self.object_ordering + [obj_idx]) in self.explored:
                ### this new ordering has been explored before and turns out to be failure
                ### no need to do it again
                continue
            ### the object has not been considered given the object_ordering, so let's check this object
            ### before we start, let's book keep 
            ### (1) the object's current pose as well as (2) the robot's current configuration
            object_curr_pos = self.serviceCall_getCertainObjectPose(obj_idx)
            robot_curr_config = self.serviceCall_getCurrRobotConfig()
            rearrange_success, object_path = self.serviceCall_rearrangeCylinderObject(obj_idx, "Right_torso")
            if rearrange_success:
                self.object_ordering.append(obj_idx)
                self.object_paths.append(object_path)
                ### recursive call
                FLAG = self.DFS_DP()
                if FLAG: 
                    return FLAG
                else:
                    ### put the object and robot back to the configuration they belong to
                    ### at the beginning of the function call
                    update_success = self.serviceCall_updateCertainObjectPose(obj_idx, object_curr_pos)
                    update_success = self.serviceCall_resetRobotCurrConfig(robot_curr_config)
                    update_success = self.serviceCall_updateManipulationStatus("Right_torso")
            else:
                ### put the object and robot back to the configuration they belong to
                ### at the beginning of the function call
                update_success = self.serviceCall_updateCertainObjectPose(obj_idx, object_curr_pos)
                update_success = self.serviceCall_resetRobotCurrConfig(robot_curr_config)
                update_success = self.serviceCall_updateManipulationStatus("Right_torso")

        ### if there is no option, before returning back
        ### pop the last element on the object_ordering
        if self.object_ordering != []:
            ### before pop out, you need to remember that
            ### you have fully explored the subtree (subproblem)
            ### which has the current ordering as the root
            ### you need to mark it as explored
            self.explored.append(set(self.object_ordering)) 
            self.object_ordering.pop(-1)
            self.object_paths.pop(-1)

        return FLAG


def main(args):
    dfsdp_task_planner = DFSDPplanner(args)
    dfsdp_task_planner.rosInit()
    rate = rospy.Rate(10) ### 10hz

    initialize_instance_success = dfsdp_task_planner.serviceCall_generateInstanceCylinder()
    if initialize_instance_success:
        cylinder_objects = dfsdp_task_planner.serviceCall_cylinderPositionEstimate()
        reproduce_instance_success = dfsdp_task_planner.serviceCall_reproduceInstanceCylinder(cylinder_objects)
        input("WELCOME TO DFS_DP PLANNER TERRITORY")
        start_time = time.time()
        TASK_SUCCESS = dfsdp_task_planner.DFS_DP()
        print("Time for planning is: {}".format(time.time() - start_time))

if __name__ == '__main__':
    main(sys.argv)