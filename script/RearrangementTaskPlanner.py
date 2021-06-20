#!/usr/bin/env python
from __future__ import division

import time
import sys
import os
import numpy as np

import rospy
import rospkg

from uniform_object_rearrangement.msg import CylinderObj
from uniform_object_rearrangement.msg import ObjectRearrangePath
from uniform_object_rearrangement.srv import GenerateInstanceCylinder, GenerateInstanceCylinderRequest
from uniform_object_rearrangement.srv import CylinderPositionEstimate, CylinderPositionEstimateRequest
from uniform_object_rearrangement.srv import ReproduceInstanceCylinder, ReproduceInstanceCylinderRequest
from uniform_object_rearrangement.srv import RearrangeCylinderObject, RearrangeCylinderObjectRequest
from uniform_object_rearrangement.srv import ExecuteTrajectory, ExecuteTrajectoryRequest

############################### description ################################
### This class defines a RearrangementTaskPlanner class which
### solves an rearrangement problems with the number of the object specified
### It
### (1) asks the execution scene to generate an instance
### (2) asks the pose estimator to get the object poses
### (3) reproduces the instance in task planner and solve it with our planner
### (4) asks the planning scene to plan the manipulation paths
### (5) asks the execution scene to execute the planned paths
#############################################################################

class RearrangementTaskPlanner(object):

    def __init__(self, args):
        ### read in relevant ros parameters for rearrangement planner
        table_dim, table_offset_x, cylinder_radius, \
        constrained_area_dim, back_distance = self.readROSParam()
        ### set the rospkg path
        rospack = rospkg.RosPack()
        self.rosPackagePath = rospack.get_path("uniform_object_rearrangement")
        self.num_objects = int(args[1])

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
            print(" execute_trajectory service call failed: %s" % e)
        

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
            self.serviceCall_execute_trajectory(path.transit_trajectory)
            ### now attach the object
            

        

    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initializes a ros node
        rospy.init_node("rearrangement_task_planner", anonymous=True)

    def readROSParam(self):
        ### This functions read in needed ROS parameters
        while not rospy.has_param('/workspace_table/table_dim'):
            rospy.sleep(0.2)
        table_dim = rospy.get_param('/workspace_table/table_dim')

        while not rospy.has_param('/workspace_table/table_offset_x'):
            rospy.sleep(0.2)
        table_offset_x = rospy.get_param('/workspace_table/table_offset_x')

        while not rospy.has_param('/uniform_cylinder_object/radius'):
            rospy.sleep(0.2)
        cylinder_radius = rospy.get_param('/uniform_cylinder_object/radius')

        while not rospy.has_param('/constrained_area/constrained_area_dim'):
            rospy.sleep(0.2)
        constrained_area_dim = rospy.get_param('/constrained_area/constrained_area_dim')

        while not rospy.has_param('/constrained_area/back_distance'):
            rospy.sleep(0.2)
        back_distance = rospy.get_param('/constrained_area/back_distance')

        return table_dim, table_offset_x, cylinder_radius, \
            constrained_area_dim, back_distance


def main(args):
    rearrangement_task_planner = RearrangementTaskPlanner(args)
    rearrangement_task_planner.rosInit()
    rate = rospy.Rate(10) ### 10hz

    initialize_instance_success = rearrangement_task_planner.serviceCall_generateInstanceCylinder()
    cylinder_objects = rearrangement_task_planner.serviceCall_cylinderPositionEstimate()
    reproduce_instance_success = rearrangement_task_planner.serviceCall_reproduceInstanceCylinder(cylinder_objects)

    # object_ordering = input('give me an object ordering')
    # object_ordering = str(object_ordering)
    # object_ordering = object_ordering.split(",")
    # object_ordering = [int(i) for i in object_ordering]
    # print(object_ordering)
    # object_ordering = [2, 1, 0]
    object_ordering = [2]
    whole_path = []
    # obj_idx = 2
    for obj_idx in object_ordering:
        rearrange_success, object_path = rearrangement_task_planner.serviceCall_rearrangeCylinderObject(obj_idx, "Right_torso")
        if not rearrange_success:
            print("oh ow, you failed at object: {}".format(obj_idx))
            break
        ### otherwise get the path
        whole_path.append(object_path)

    execute_success = rearrangement_task_planner.executeWholePlan(whole_path)
    if execute_success: 
        rospy.logwarn("THE REARRANGEMENT TASK IS FULFILLED BY THE ROBOT")
    else:
        rospy.logwarn("THE REARRANGEMENT TASK IS NOT FULFILLED BY THE ROBOT")


    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)



########################## below is not used but kept for legacy ##########################
# for cylinder_object in cylinder_objects:
#     obj_idx = cylinder_object.obj_idx
#     start_pos = [cylinder_object.start_position.x, cylinder_object.start_position.y, cylinder_object.start_position.z]
#     goal_pos = [cylinder_object.goal_position.x, cylinder_object.goal_position.y, cylinder_object.goal_position.z]
#     print(obj_idx)
#     print(start_pos)
#     print(goal_pos)
