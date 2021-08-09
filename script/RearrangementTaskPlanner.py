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

from uniform_object_rearrangement.srv import RearrangeCylinderObject, RearrangeCylinderObjectRequest
from uniform_object_rearrangement.srv import GetCertainObjectPose, GetCertainObjectPoseRequest
from uniform_object_rearrangement.srv import GetCurrRobotConfig, GetCurrRobotConfigRequest
from uniform_object_rearrangement.srv import UpdateCertainObjectPose, UpdateCertainObjectPoseRequest
from uniform_object_rearrangement.srv import ResetRobotCurrConfig, ResetRobotCurrConfigRequest
from uniform_object_rearrangement.srv import UpdateManipulationStatus, UpdateManipulationStatusRequest

class RearrangementTaskPlanner(object):
    def __init__(self):
        ### set the rospkg path
        rospy.logwarn("INITIALIZE A REARRANGEMENT TASK PLANNER")
        rospack = rospkg.RosPack()
        self.rosPackagePath = rospack.get_path("uniform_object_rearrangement")

    def setPlanningParams(self, nums_objects, isLabeledRoadmapUsed=True):
        self.num_objects = nums_objects
        self.all_objects = [i for i in range(self.num_objects)]
        self.object_ordering = [] ### a list of obj_idx
        self.object_paths = [] ### a list of ObjectRearrangePath paths
        self.time_threshold = 180 ### 180s
        self.planning_startTime = time.time()
        self.isLabeledRoadmapUsed = isLabeledRoadmapUsed

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
            object_curr_position_idx = getObjectPose_response.curr_position_idx
            return object_curr_pos, object_curr_position_idx
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

    def serviceCall_rearrangeCylinderObject(self, obj_idx, armType, isLabeledRoadmapUsed=True):
        rospy.wait_for_service("rearrange_cylinder_object")
        request = RearrangeCylinderObjectRequest()
        request.object_idx = obj_idx
        request.armType = armType
        request.isLabeledRoadmapUsed = isLabeledRoadmapUsed
        try:
            rearrangeCylinderObject_proxy = rospy.ServiceProxy(
                "rearrange_cylinder_object", RearrangeCylinderObject)
            rearrange_cylinder_object_response = rearrangeCylinderObject_proxy(request)
            return rearrange_cylinder_object_response.success, rearrange_cylinder_object_response.path
        except rospy.ServiceException as e:
            print("rearrange_cylinder_object service call failed: %s" % e)

    def serviceCall_updateCertainObjectPose(self, obj_idx, target_pose, target_position_idx):
        '''call the UpdateCertainObjectPose service to update the object
           to the specified target pose'''
        rospy.wait_for_service("update_certain_object_pose")
        request = UpdateCertainObjectPoseRequest()
        request.object_idx = obj_idx
        request.target_pose = Point(target_pose[0], target_pose[1], target_pose[2])
        request.object_position_idx = target_position_idx
        try:
            updateCertainObjectPose_proxy = rospy.ServiceProxy(
                                            "update_certain_object_pose", UpdateCertainObjectPose)
            updateCertainObjectPose_response = updateCertainObjectPose_proxy(
                                request.object_idx, request.target_pose, request.object_position_idx)
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

    def DFS(self):
        '''search for the remaining object ordering based on current object ordering'''
        ### (1) update self.object_ordering and self.paths (a list of ObjectRearrangePath paths)
        ### (2) return FLAG==true if the problem is solved by mRS (an indication of monotonicity)

        ### first check time constraint
        # if time.time() - self.planning_startTime >= self.time_threshold:
        #     return False
        print("object_ordering: " + str(self.object_ordering))
        FLAG = False
        ### check the base case: object_ordering has been fully filled
        if (len(self.object_ordering) == self.num_objects):
            return True
        for obj_idx in self.all_objects:
            if obj_idx in self.object_ordering:
                continue
            else:
                ### the object has not been considered given the object_ordering, so let's check this object
                ### before we start, let's book keep 
                ### (1) the object's current pose as well as (2) the robot's current configuration
                object_curr_pos, object_curr_position_idx = self.serviceCall_getCertainObjectPose(obj_idx)
                robot_curr_config = self.serviceCall_getCurrRobotConfig()
                rearrange_success, object_path = self.serviceCall_rearrangeCylinderObject(
                                                    obj_idx, "Right_torso", isLabeledRoadmapUsed=self.isLabeledRoadmapUsed)
                if rearrange_success:
                    self.object_ordering.append(obj_idx)
                    self.object_paths.append(object_path)
                    ### recursive call
                    FLAG = self.DFS()
                    if FLAG: 
                        return FLAG
                    else:
                        ### put the object and robot back to the configuration they belong to
                        ### at the beginning of the function call
                        update_success = self.serviceCall_updateCertainObjectPose(
                                                obj_idx, object_curr_pos, object_curr_position_idx)
                        update_success = self.serviceCall_resetRobotCurrConfig(robot_curr_config)
                        update_success = self.serviceCall_updateManipulationStatus("Right_torso")
                else:
                    ### put the object and robot back to the configuration they belong to
                    ### at the beginning of the function call
                    update_success = self.serviceCall_updateCertainObjectPose(
                                            obj_idx, object_curr_pos, object_curr_position_idx)
                    update_success = self.serviceCall_resetRobotCurrConfig(robot_curr_config)
                    update_success = self.serviceCall_updateManipulationStatus("Right_torso")

        ### if there is no option, before returning back
        ### pop the last element on the object_ordering
        if self.object_ordering != []:
            self.object_ordering.pop(-1)
            self.object_paths.pop(-1)

        return FLAG

    def DFS_DP(self):
        '''search for the remaining object ordering based on current object ordering
           but it remembers all the explored status'''
        ### (1) update self.object_ordering and self.paths (a list of ObjectRearrangePath paths)
        ### (2) increment the explored list (item: set())
        ### (3) return FLAG==true if the problem is solved by DFS_DP (an indication of monotonicity)

        ### first check time constraint
        # if time.time() - self.planning_startTime >= self.time_threshold:
        #     return False
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
            object_curr_pos, object_curr_position_idx = self.serviceCall_getCertainObjectPose(obj_idx)
            robot_curr_config = self.serviceCall_getCurrRobotConfig()
            rearrange_success, object_path = self.serviceCall_rearrangeCylinderObject(
                                                    obj_idx, "Right_torso", isLabeledRoadmapUsed=self.isLabeledRoadmapUsed)
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
                    update_success = self.serviceCall_updateCertainObjectPose(
                                            obj_idx, object_curr_pos, object_curr_position_idx)
                    update_success = self.serviceCall_resetRobotCurrConfig(robot_curr_config)
                    update_success = self.serviceCall_updateManipulationStatus("Right_torso")
            else:
                ### put the object and robot back to the configuration they belong to
                ### at the beginning of the function call
                update_success = self.serviceCall_updateCertainObjectPose(
                                        obj_idx, object_curr_pos, object_curr_position_idx)
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

    def DFS_DP_task_planning(self, nums_objects, isLabeledRoadmapUsed=True):
        self.setPlanningParams(nums_objects, isLabeledRoadmapUsed)
        self.explored = [] ### a list of set() - current object set (e.g, (1,2,3) == (3,2,1))
        TASK_SUCCESS = self.DFS_DP()
        return TASK_SUCCESS, self.object_ordering

    def mRS_task_planning(self, nums_objects, isLabeledRoadmapUsed=True):
        self.setPlanningParams(nums_objects, isLabeledRoadmapUsed)
        TASK_SUCCESS = self.DFS()
        return TASK_SUCCESS, self.object_ordering


def main(args):
    rospy.logwarn("YOU ARE TOUCHING THE MAIN FUNCTION OF REARRANGEMENT TASK PLANNER!!!")

if __name__ == '__main__':
    main(sys.argv)