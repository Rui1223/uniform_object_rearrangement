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
from uniform_object_rearrangement.srv import AttachObject, AttachObjectRequest

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
        rospy.init_node("rearrangement_task_planner", anonymous=True)


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
    object_ordering = [2, 1, 0]
    # object_ordering = [2]
    whole_path = []
    TASK_SUCCESS = True
    # obj_idx = 2
    start_time = time.time()
    for obj_idx in object_ordering:
        rearrange_success, object_path = rearrangement_task_planner.serviceCall_rearrangeCylinderObject(obj_idx, "Right_torso")
        if not rearrange_success:
            print("oh ow, you failed at object: {}".format(obj_idx))
            TASK_SUCCESS = False
            break
        ### otherwise get the path
        whole_path.append(object_path)
    end_time = time.time()
    print("Time for planning is: {}".format(end_time - start_time))

    if TASK_SUCCESS:
        start_time = time.time()
        execute_success = rearrangement_task_planner.executeWholePlan(whole_path)
        if execute_success: 
            rospy.logwarn("THE REARRANGEMENT TASK IS FULFILLED BY THE ROBOT")
        else:
            rospy.logwarn("THE REARRANGEMENT TASK IS NOT FULFILLED BY THE ROBOT")
        end_time = time.time()
        print("Time for executing is: {}".format(end_time - start_time))


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
