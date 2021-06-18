#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import time
import sys
import os
import copy
import math
import numpy as np

import rospy
import rospkg
from std_msgs.msg import String

from MotomanRobot import MotomanRobot
from WorkspaceTable import WorkspaceTable
from Planner import Planner

from uniform_object_rearrangement.msg import CylinderObj
from uniform_object_rearrangement.srv import ReproduceInstanceCylinder, ReproduceInstanceCylinderResponse
from uniform_object_rearrangement.srv import RearrangeCylinderObject, RearrangeCylinderObjectResponse


################################## description #####################################
### This class defines a PybulletPlanScene class which
### entertain an planning scene that
### (1) reproduces the instance from the execution scene based on perception output
### (2) performs planning for manipulation (transit + transfer)
### (3) communicates with grasp_pose node to get the grasp pose for grasping objects
####################################################################################

class PybulletPlanScene(object):

    def __init__(self, args):
        ### read in relevant ros parameters for plan scene
        basePosition, baseOrientation, urdfFile, \
        leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration, \
        standingBase_dim, table_dim, table_offset_x, \
        self.cylinder_radius, self.cylinder_height, \
        constrained_area_dim, thickness_flank, back_distance, \
        object_mesh_path = self.readROSParam()
        
        print("1111111111111")
        ### set the rospkg path
        rospack = rospkg.RosPack()
        self.rosPackagePath = rospack.get_path("uniform_object_rearrangement")
        print("1111111111111")
	
        ### set the server for the pybullet plan scene
        # self.planningClientID = p.connect(p.DIRECT)
        self.planningClientID = p.connect(p.GUI)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # self.egl_plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        # print("plugin=", self.egl_plugin)
        print("1111111111111")
        ### create a planner assistant
        self.planner_p = Planner(
            self.rosPackagePath, self.planningClientID,
            isObjectInLeftHand=False, isObjectInRightHand=False,
            objectInLeftHand=None, objectInRightHand=None)

        ### configure the robot
        self.configureMotomanRobot(urdfFile, basePosition, baseOrientation, \
            leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration, False)
        ### setup the workspace
        self.setupWorkspace(standingBase_dim, table_dim, table_offset_x, object_mesh_path, False)
        self.workspace_p.addConstrainedArea(constrained_area_dim, thickness_flank, back_distance)


    def configureMotomanRobot(self, 
            urdfFile, basePosition, baseOrientation,
            leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration, isPhysicsTurnOn):
        ### This function configures the robot in the real scene ###
        self.robot_p = MotomanRobot(
            os.path.join(self.rosPackagePath, urdfFile), 
            basePosition, baseOrientation, 
            leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration,
            isPhysicsTurnOn, self.planningClientID)

    def setupWorkspace(self,
            standingBase_dim, table_dim, table_offset_x,
            object_mesh_path, isPhysicsTurnOn):
        ### This function sets up the workspace ###
        self.workspace_p = WorkspaceTable(self.robot_p.basePosition,
            standingBase_dim, table_dim, table_offset_x, 
            os.path.join(self.rosPackagePath, object_mesh_path),
            isPhysicsTurnOn, self.planningClientID)

    def rosInit(self):
        ### This function specifies the role of a node instance for this class ###
        ### and initialize a ros node
        self.reproduce_instance_cylinder_server = rospy.Service(
            "reproduce_instance_cylinder", ReproduceInstanceCylinder,
            self.reproduce_instance_cylinder_callback)
        self.rearrange_cylinder_object = rospy.Service(
            "rearrange_cylinder_object", RearrangeCylinderObject,
            self.rearrange_cylinder_object_callback)

        rospy.init_node("pybullet_plan_scene", anonymous=True)


    def reproduce_instance_cylinder_callback(self, req):
        ### given the estimated cylinder objects
        rospy.logwarn("REPRODUCE REARRANGEMENT INSTANCE")
        success = self.workspace_p.reproduceInstance_cylinders(req.cylinder_objects)
        if success == True:
            print("successfully reproduce an instance")
        else:
            print("fail to reproduce an instance")
        return ReproduceInstanceCylinderResponse(success)

    def getCurrentConfig(self, armType):
        if armType == "Right":
            currConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
        if armType == "Right_torso":
            currConfig = copy.deepcopy([self.robot_p.torsoCurrConfiguration] + self.robot_p.rightArmCurrConfiguration)
        return currConfig

    def provide_pose_for_object(self, object_idx, manipulation_type):
        if manipulation_type == "picking":
            targetPose = [[self.workspace_p.object_geometries[object_idx].start_pos[0] - 0.02,
                        self.workspace_p.object_geometries[object_idx].start_pos[1],
                        self.workspace_p.object_geometries[object_idx].start_pos[2]], 
                        [0.707, 0, 0.707, 0]]
        if manipulation_type == "placing":
            targetPose = [[self.workspace_p.object_geometries[object_idx].goal_pos[0] - 0.02,
                        self.workspace_p.object_geometries[object_idx].goal_pos[1],
                        self.workspace_p.object_geometries[object_idx].goal_pos[2]], 
                        [0.707, 0, 0.707, 0]]
        return targetPose


    def rearrange_cylinder_object_callback(self, req):
        ### given the specified cylinder object and the armType
        rospy.logwarn("PLANNING TO REARRANGE THE OBJECT")
        print("object: {}".format(req.object_idx))
        currConfig = self.getCurrentConfig(req.armType)
        pickingPose = self.provide_pose_for_object(req.object_idx, "picking")
        isPoseValid, configToPickingPose = self.planner_p.generateConfigBasedOnPose(
                    pickingPose, req.object_idx, self.robot_p, self.workspace_p, req.armType, "transit")

        if not isPoseValid:
            print("this pose is not even valid, let alone generating pre-picking")
            return RearrangeCylinderObjectResponse(isPoseValid)
        else:
            print("the picking pose is valid, generate pre-picking")
            isPoseValid, prePickingPose, configToPrePickingPose = self.planner_p.generatePrePickingOrPostPlacingPose(
                        pickingPose, req.object_idx, self.robot_p, self.workspace_p, req.armType, "transit")
            if not isPoseValid:
                print("the pre-picking pose is not valid, thus the picking pose is deemed as invalid as well")
                return RearrangeCylinderObjectResponse(isPoseValid)
        print("both picking pose and pre-picking pose are legitimate. Proceed to planning.")

        result_traj = self.planner_p.AstarPathFinding(currConfig, configToPrePickingPose, req.object_idx, 
                                            self.robot_p, self.workspace_p, req.armType, "transit")
        ### the planning has been finished, either success or failure
        if result_traj != []:
            print("the transit path for %s arm is successfully found" % req.armType)
        else:
            print("the transit path for %s arm is not successfully found" % req.armType)
            return RearrangeCylinderObjectResponse(False)

        currConfig = self.getCurrentConfig(req.armType)
        ### you are reaching here since pre-picking has been reached, now get the path from pre-picking to picking
        result_traj = []
        config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                            currConfig, configToPickingPose, self.robot_p, req.armType, req.object_idx, self.workspace_p)
        result_traj.append(config_edge_traj)

        ### Now we need to attach the object in hand before transferring the object
        self.planner_p.attachObject(req.object_idx, self.workspace_p, self.robot_p, req.armType)

        ### generate post-picking pose
        postPickingPose = copy.deepcopy(pickingPose)
        postPickingPose[0][2] += 0.05
        isPoseValid, configToPostPickingPose = self.planner_p.generateConfigBasedOnPose(
                    postPickingPose, req.object_idx, self.robot_p, self.workspace_p, req.armType, "transfer")

        currConfig = self.getCurrentConfig(req.armType)
        result_traj = []
        config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                            currConfig, configToPostPickingPose, self.robot_p, req.armType, req.object_idx, self.workspace_p)
        result_traj.append(config_edge_traj)

        currConfig = self.getCurrentConfig(req.armType)
        placingPose = self.provide_pose_for_object(req.object_idx, "placing")
        isPoseValid, configToPlacingPose = self.planner_p.generateConfigBasedOnPose(
                    placingPose, req.object_idx, self.robot_p, self.workspace_p, req.armType, "transfer")
        
        if not isPoseValid:
            print("this pose is not even valid, let alone generating pre-placing")
            return RearrangeCylinderObjectResponse(isPoseValid)
        else:
            print("the placing pose is valid, generate pre-placing")
            prePlacingPose = copy.deepcopy(placingPose)
            prePlacingPose[0][2] += 0.05
            isPoseValid, configToPrePlacingPose = self.planner_p.generateConfigBasedOnPose(
                        prePlacingPose, req.object_idx, self.robot_p, self.workspace_p, req.armType, "transfer")
            if not isPoseValid:
                print("the pre-placing pose is not valid, thus the placing pose is deemed as invalid as well")
                return RearrangeCylinderObjectResponse(isPoseValid)
        print("both placing pose and pre-placing pose are legitimate. Proceed to planning.")
        
        result_traj = self.planner_p.AstarPathFinding(currConfig, configToPrePlacingPose, req.object_idx,
                                            self.robot_p, self.workspace_p, req.armType, "transfer")
        ### the planning has been finished, either success or failure
        if result_traj != []:
            print("the transfer path for %s arm is successfully found" % req.armType)
        else:
            print("the transfer path for %s arm is not successfully found" % req.armType)
            return RearrangeCylinderObjectResponse(False)
        
        currConfig = self.getCurrentConfig(req.armType)
        ### you are reaching here since pre-placing has been reached, now get the path from pre-placing to placing
        result_traj = []
        config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                            currConfig, configToPlacingPose, self.robot_p, req.armType, req.object_idx, self.workspace_p)
        result_traj.append(config_edge_traj)

        # isPoseValid, configToPlacingPose = self.planner_p.generateConfigBasedOnPose(
        #             placingPose, req.object_idx, self.robot_p, self.workspace_p, req.armType, "transfer")
        # print("IS POSE VALID?????????", isPoseValid)
        # if not isPoseValid:
        #     print("this placing pose is not even valid. No motion planning will be attempted")
        #     return RearrangeCylinderObjectResponse(False)
        # print("the placing pose is legitimate. Proceed to planning.")

        # result_traj = self.planner_p.AstarPathFinding(currConfig, configToPlacingPose, req.object_idx,
        #                                     self.robot_p, self.workspace_p, req.armType, "transfer")
        # if result_traj != []:
        #     print("the transfer path for %s arm is successfully found" % req.armType)
        # else:
        #     print("the transfer path for %s arm is not successfully found" % req.armType)
        #     return RearrangeCylinderObjectResponse(False)

        ### detach the object
        self.planner_p.detachObject(self.workspace_p, self.robot_p, req.armType)
        ### retract the arm from the object
        currConfig = self.getCurrentConfig(req.armType)
        isPoseValid, postPlacingPose, configToPostPlacingPose = self.planner_p.generatePrePickingOrPostPlacingPose(
                    placingPose, req.object_idx, self.robot_p, self.workspace_p, req.armType, "transit")
        if not isPoseValid:
            print("The post-placing pose is not valid")
            return RearrangeCylinderObjectResponse(isPoseValid)
        print("post-placing is legitimate. Proceed to planning.")

        ### we need to check if a transition from targetPose to post-placement pose will work
        result_traj = []
        config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
                            currConfig, configToPostPlacingPose, self.robot_p, req.armType, req.object_idx, self.workspace_p)
        result_traj.append(config_edge_traj)

        currConfig = self.getCurrentConfig(req.armType)

        return RearrangeCylinderObjectResponse(True)



    def readROSParam(self):
        ### This functions read in needed ROS parameters
        while not rospy.has_param('/motoman_robot/basePosition'):
            rospy.sleep(0.2)
        basePosition = rospy.get_param('/motoman_robot/basePosition')

        while not rospy.has_param('/motoman_robot/baseOrientation'):
            rospy.sleep(0.2)
        baseOrientation = rospy.get_param('/motoman_robot/baseOrientation')

        while not rospy.has_param('/motoman_robot/urdfFile'):
            rospy.sleep(0.2)
        urdfFile = rospy.get_param('/motoman_robot/urdfFile')

        while not rospy.has_param('/motoman_robot/leftArmHomeConfiguration'):
            rospy.sleep(0.2)
        leftArmHomeConfiguration = rospy.get_param('/motoman_robot/leftArmHomeConfiguration')

        while not rospy.has_param('/motoman_robot/rightArmHomeConfiguration'):
            rospy.sleep(0.2)
        rightArmHomeConfiguration = rospy.get_param('/motoman_robot/rightArmHomeConfiguration')

        while not rospy.has_param('/motoman_robot/torsoHomeConfiguration'):
            rospy.sleep(0.2)
        torsoHomeConfiguration = rospy.get_param('/motoman_robot/torsoHomeConfiguration')

        while not rospy.has_param('/workspace_table/standingBase_dim'):
            rospy.sleep(0.2)
        standingBase_dim = rospy.get_param('/workspace_table/standingBase_dim')

        while not rospy.has_param('/workspace_table/table_dim'):
            rospy.sleep(0.2)
        table_dim = rospy.get_param('/workspace_table/table_dim')

        while not rospy.has_param('/workspace_table/table_offset_x'):
            rospy.sleep(0.2)
        table_offset_x = rospy.get_param('/workspace_table/table_offset_x')

        while not rospy.has_param('/uniform_cylinder_object/radius'):
            rospy.sleep(0.2)
        cylinder_radius = rospy.get_param('/uniform_cylinder_object/radius')

        while not rospy.has_param('/uniform_cylinder_object/height'):
            rospy.sleep(0.2)
        cylinder_height = rospy.get_param('/uniform_cylinder_object/height')

        while not rospy.has_param('/constrained_area/constrained_area_dim'):
            rospy.sleep(0.2)
        constrained_area_dim = rospy.get_param('/constrained_area/constrained_area_dim')

        while not rospy.has_param('/constrained_area/thickness_flank'):
            rospy.sleep(0.2)
        thickness_flank = rospy.get_param('/constrained_area/thickness_flank')

        while not rospy.has_param('/constrained_area/back_distance'):
            rospy.sleep(0.2)
        back_distance = rospy.get_param('/constrained_area/back_distance')

        while not rospy.has_param('/object_mesh_to_drop_in_real_scene/object_mesh_path'):
            rospy.sleep(0.2)
        object_mesh_path = rospy.get_param('/object_mesh_to_drop_in_real_scene/object_mesh_path')

        return basePosition, baseOrientation, urdfFile, \
            leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration, \
            standingBase_dim, table_dim, table_offset_x, \
            cylinder_radius, cylinder_height, \
            constrained_area_dim, thickness_flank, back_distance, \
            object_mesh_path


def main(args):
    pybullet_plan_scene = PybulletPlanScene(args)
    pybullet_plan_scene.planner_p.loadSamples()
    pybullet_plan_scene.rosInit()
    rate = rospy.Rate(10) ### 10hz

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)



##################################### temporarily kept here #####################################
### we need to check if a transition from pre-grasp to final grasp will work
# isDirectPathValid = self.planner_p.checkEdgeValidity_DirectConfigPath(
#                         configToPreGraspPose, configToGraspPose, req.object_idx, 
#                         self.robot_p, self.workspace_p, req.armType, "transit")
# if isDirectPathValid:
#     result_traj = []
#     config_edge_traj = self.planner_p.generateTrajectory_DirectConfigPath(
#                         configToPreGraspPose, configToGraspPose, self.robot_p, req.armType, req.object_idx, self.workspace_p)
#     result_traj.append(config_edge_traj)

# if result_traj != []:
#     print("the pregrasp->grasp path for %s arm is successfully found" % req.armType)
# else:
#     print("the pregrasp->grasp path for %s arm is not successfully found" % req.armType)
#     return RearrangeCylinderObjectResponse(False)
