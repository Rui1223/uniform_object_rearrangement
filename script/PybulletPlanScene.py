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
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from MotomanRobot import MotomanRobot
from WorkspaceTable import WorkspaceTable
from Planner import Planner
import utils

from uniform_object_rearrangement.msg import ArmTrajectory
from uniform_object_rearrangement.msg import ObjectRearrangePath
from uniform_object_rearrangement.srv import ReproduceInstanceCylinder, ReproduceInstanceCylinderResponse
from uniform_object_rearrangement.srv import RearrangeCylinderObject, RearrangeCylinderObjectResponse
from uniform_object_rearrangement.srv import GetCertainObjectPose, GetCertainObjectPoseResponse
from uniform_object_rearrangement.srv import GetCurrRobotConfig, GetCurrRobotConfigResponse
from uniform_object_rearrangement.srv import UpdateCertainObjectPose, UpdateCertainObjectPoseResponse
from uniform_object_rearrangement.srv import ResetRobotCurrConfig, ResetRobotCurrConfigResponse
from uniform_object_rearrangement.srv import UpdateManipulationStatus, UpdateManipulationStatusResponse

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
        cylinder_radius, cylinder_height, \
        discretization_x, discretization_y, \
        object_interval_x, object_interval_y, \
        side_clearance_x, side_clearance_y, \
        ceiling_height, thickness_flank, \
        object_mesh_path = self.readROSParam()
        
        ### set the rospkg path
        rospack = rospkg.RosPack()
        self.rosPackagePath = rospack.get_path("uniform_object_rearrangement")
	
        ### set the server for the pybullet plan scene
        # self.planningClientID = p.connect(p.DIRECT)
        self.planningClientID = p.connect(p.GUI)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # self.egl_plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        # print("plugin=", self.egl_plugin)

        ### configure the robot
        self.configureMotomanRobot(urdfFile, basePosition, baseOrientation, \
            leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration, False)
        ### setup the workspace
        self.setupWorkspace(standingBase_dim, table_dim, table_offset_x, object_mesh_path, False)
        self.workspace_p.addConstrainedArea(ceiling_height, thickness_flank)
        self.workspace_p.setDeploymentParam(
                cylinder_radius, cylinder_height, discretization_x, discretization_y, \
                object_interval_x, object_interval_y, side_clearance_x, side_clearance_y)
        self.workspace_p.deployAllPositionCandidates()

        ### create a planner assistant
        self.planner_p = Planner(
            self.rosPackagePath, self.planningClientID,
            isObjectInLeftHand=False, isObjectInRightHand=False,
            objectInLeftHand=None, objectInRightHand=None)


    def configureMotomanRobot(self, 
            urdfFile, basePosition, baseOrientation,
            leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration, isPhysicsTurnOn):
        ### This function configures the robot in the real scene
        self.robot_p = MotomanRobot(
            os.path.join(self.rosPackagePath, urdfFile), 
            basePosition, baseOrientation, 
            leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration,
            isPhysicsTurnOn, self.planningClientID)

    def setupWorkspace(self,
            standingBase_dim, table_dim, table_offset_x,
            object_mesh_path, isPhysicsTurnOn):
        ### This function sets up the workspace
        self.workspace_p = WorkspaceTable(self.rosPackagePath, self.robot_p.basePosition,
            standingBase_dim, table_dim, table_offset_x, 
            os.path.join(self.rosPackagePath, object_mesh_path),
            isPhysicsTurnOn, self.planningClientID)

    def deployAllGoalPositions(self, object_interval_x=None, object_interval_y=None):
        ### ask workspace to deploy all goal positions
        self.workspace_p.deployAllGoalPositions(object_interval_x, object_interval_y)


    def rosInit(self):
        ### This function specifies the role of a node instance for this class
        ### and initialize a ros node
        self.reproduce_instance_cylinder_server = rospy.Service(
            "reproduce_instance_cylinder", ReproduceInstanceCylinder,
            self.reproduce_instance_cylinder_callback)
        self.rearrange_cylinder_object_server = rospy.Service(
            "rearrange_cylinder_object", RearrangeCylinderObject,
            self.rearrange_cylinder_object_callback_multiposes_improved)

        self.get_certain_object_pose_server = rospy.Service(
            "get_certain_object_pose", GetCertainObjectPose,
            self.get_certain_object_pose_callback)

        self.get_curr_robot_config_server = rospy.Service(
            "get_curr_robot_config", GetCurrRobotConfig,
            self.get_curr_robot_config_callback)

        self.update_certain_object_pose_server = rospy.Service(
            "update_certain_object_pose", UpdateCertainObjectPose,
            self.update_certain_object_pose_callback)

        self.reset_robot_curr_config_server = rospy.Service(
            "reset_robot_curr_config", ResetRobotCurrConfig,
            self.reset_robot_curr_config_callback)

        self.update_manipulation_status_server = rospy.Service(
            "update_manipulation_status", UpdateManipulationStatus,
            self.update_manipulation_status_callback)

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

    def get_certain_object_pose_callback(self, req):
        ### given the specified object index
        temp_curr_pos = self.workspace_p.object_geometries[req.object_idx].curr_pos
        curr_position = Point(temp_curr_pos[0], temp_curr_pos[1], temp_curr_pos[2])
        # print("successfully get the object pose for object " + str(req.object_idx))
        return GetCertainObjectPoseResponse(curr_position)

    def get_curr_robot_config_callback(self, req):
        ### get the current robot config
        joint_state = JointState()
        joint_state.position = self.robot_p.getRobotCurrConfig()
        # print("successfully get the current robot configuration")
        return GetCurrRobotConfigResponse(joint_state)

    def update_certain_object_pose_callback(self, req):
        ### update the geometry mesh of a certain object to the target pose
        position = [req.target_pose.x, req.target_pose.y, req.target_pose.z]
        self.workspace_p.updateObjectMesh(req.object_idx, position)
        # print("successfully update certain object geometry to the target pose" + str(position))
        return UpdateCertainObjectPoseResponse(True)

    def reset_robot_curr_config_callback(self, req):
        ### reset the robot to the specified configuration
        joint_positions = list(req.robot_config.position)
        self.robot_p.resetArmConfig_torso(joint_positions[1:15], joint_positions[0])
        ### update the hand as well (I doubt it is a redundant step at this point)
        self.robot_p.resetRightHandConfig(joint_positions[15:21])
        # print("successfully reset the robot current configuration")
        return ResetRobotCurrConfigResponse(True)

    def update_manipulation_status_callback(self, req):
        ### disable any relationship between the robot and any of the object
        self.planner_p.detachObject(self.workspace_p, self.robot_p, req.armType)
        # print("successfully update the manipulation status")
        return UpdateManipulationStatusResponse(True)


    def getCurrentConfig(self, armType):
        if armType == "Right":
            currConfig = copy.deepcopy(self.robot_p.rightArmCurrConfiguration)
        if armType == "Right_torso":
            currConfig = copy.deepcopy([self.robot_p.torsoCurrConfiguration] + self.robot_p.rightArmCurrConfiguration)
        return currConfig

    
    def generate_pose_candidates(self, position):
        ### position: [x,y,z]
        pose_candidates = []
        orientations = self.generateOrientations()

        ### for each orientation
        for orientation in orientations:
            targetPose = [[position[0], position[1], position[2] + self.workspace_p.cylinder_height/2 - 0.03], orientation]
            pose_candidates.append(targetPose)

        return pose_candidates


    def generateArmTrajectory(self, traj, armType, motomanRJointNames):
        '''generate arm trajectory (a list of JointState)
        inputs
        ======
            traj (a list of list): a list of joint states [q1, q2, ..., qn]
            armType (string): the arm type (e.g., "Left", "Right_torso)
            motomanRJointNames (a list of strings): the names for controllable joints
        outputs
        =======
            result_traj (ArmTrajectory()): the resulting trajectory (ArmTrajectory object)
        '''
        result_traj = ArmTrajectory()
        result_traj.armType = armType
        if armType == "Left" or armType == "Left_torso":
            first_joint_index = 1
        if armType == "Right" or armType == "Right_torso":
            first_joint_index = 8
        if armType == "Left_torso" or armType == "Right_torso":
            jointNames = [motomanRJointNames[0]] + motomanRJointNames[first_joint_index:first_joint_index+7]
        if armType == "Left" or armType == "Right":
            jointNames = motomanRJointNames[first_joint_index:first_joint_index+7]

        for config in traj:
            joint_state = JointState()
            joint_state.name = jointNames
            joint_state.position = config
            result_traj.trajectory.append(joint_state)
        
        return result_traj

    def rearrange_cylinder_object_callback_multiposes_improved(self, req):
        ### given the specified cylinder object and the armType
        rospy.logwarn("PLANNING TO REARRANGE THE OBJECT")
        print("object: {}".format(req.object_idx))
        object_path = ObjectRearrangePath()
        transit_traj = []
        transfer_traj = []
        finish_traj = []

        currConfig = self.getCurrentConfig(req.armType)
        ########################## generate picking pose candidates ###################################
        pickingPose_candidates = self.generate_pose_candidates(
                                        self.workspace_p.object_geometries[req.object_idx].curr_pos)
        ###############################################################################################

        #################### select the right picking pose until it works #############################
        transit_success = False
        for pose_id, pickingPose in enumerate(pickingPose_candidates):
            ######################## check both picking and pre-picking pose ##########################
            isPoseValid, FLAG, configToPickingPose = self.planner_p.generateConfigBasedOnPose(
                                pickingPose, currConfig, self.robot_p, self.workspace_p, req.armType)
            if not isPoseValid:
                print("This picking pose is not even valid. Move on to next candidate.")
                continue
            else:
                ### check the connection with neighbors in the roadmap
                print("The picking pose works. Check its neighboring connections.")
                ### when to check the connection of the picking pose, you have to attach the object
                temp_object_curr_pos = self.workspace_p.object_geometries[req.object_idx].curr_pos
                self.planner_p.attachObject(req.object_idx, self.workspace_p, self.robot_p, req.armType)
                connectSuccess, pickingPose_neighbors_idx, pickingPose_neighbors_cost = self.planner_p.connectToNeighbors(
                                            configToPickingPose, self.robot_p, self.workspace_p, req.armType)
                ############## after check, disattach the object and put the object back ##############
                self.planner_p.detachObject(self.workspace_p, self.robot_p, req.armType)
                p.resetBasePositionAndOrientation(
                    self.workspace_p.object_geometries[req.object_idx].geo, 
                    temp_object_curr_pos, [0, 0, 0, 1.0], physicsClientId=self.planningClientID)
                self.workspace_p.object_geometries[req.object_idx].curr_pos = temp_object_curr_pos
                #######################################################################################

                if not connectSuccess:
                    print("This picking pose is not valid, due to no neighboring connections.")
                    print("Move on to next candidate.")
                    continue
                else:                    
                    print("The picking pose is valid, generate pre-picking")
                    isPoseValid, FLAG, prePickingPose, configToPrePickingPose = \
                        self.planner_p.generatePrePickingPose(
                            pickingPose, currConfig, self.robot_p, self.workspace_p, req.armType)
                    if not isPoseValid:
                        print("The pre-picking pose is not valid. Move on to next candidate.")
                        continue
                    else:
                        ### check the connection with neighbors in the roadmap
                        print("The pre-picking pose works. Check its neighboring connections.")
                        connectSuccess, prePickingPose_neighbors_idx, prePickingPose_neighbors_cost = self.planner_p.connectToNeighbors(
                                    configToPrePickingPose, self.robot_p, self.workspace_p, req.armType)
                        if not connectSuccess:
                            print("This pre-picking pose is not valid, due to no neighboring connections.")
                            print("Move on to next candidate.")
                            continue
                        print("Both picking pose and pre-picking pose are legitimate. Proceed to planning for pre-picking.")
            ###########################################################################################

            ################### plan the path to pre-picking configuration ############################
            connectSuccess, currConfig_neighbors_idx, currConfig_neighbors_cost = self.planner_p.connectToNeighbors(
                        currConfig, self.robot_p, self.workspace_p, req.armType)
            prePicking_traj = self.planner_p.AstarPathFinding_new(currConfig, configToPrePickingPose, 
                                currConfig_neighbors_idx, currConfig_neighbors_cost, 
                                prePickingPose_neighbors_idx, prePickingPose_neighbors_cost, 
                                self.robot_p, self.workspace_p, req.armType)
            ### the planning has been finished, either success or failure
            if prePicking_traj != []:
                print("The transit (pre-picking) path for %s arm is successfully found" % req.armType)
                transit_traj += prePicking_traj
                ################# cartesian path from pre-picking to picking configuration #####################
                currConfig = self.getCurrentConfig(req.armType)
                ### you are reaching here since pre-picking has been reached, 
                ### now get the path from pre-picking to picking
                prePickToPickTraj = self.planner_p.generateTrajectory_DirectConfigPath(
                    currConfig, configToPickingPose, self.robot_p, req.armType, self.workspace_p)
                transit_traj += prePickToPickTraj
                #################################################################################################
                transit_success = True
                break
            else:
                print("The transit (pre-picking) path for %s arm is not successfully found" % req.armType)
                print("Move on to next candidate")
                continue
            ###########################################################################################
        
        if not transit_success:
            print("No picking pose is qualified, either failed (1) picking pose (2) pre-picking pose (3) planning to pre-picking")
            return RearrangeCylinderObjectResponse(False, object_path)
        
        ### Otherwise, congrats! Transit is successful!
        ######################################### attach the object ###########################################
        ### Now we need to attach the object in hand before transferring the object
        self.planner_p.attachObject(req.object_idx, self.workspace_p, self.robot_p, req.armType)
        #######################################################################################################

        currConfig = self.getCurrentConfig(req.armType)
        ########################## generate placing pose candidates ###################################
        placingPose_candidates = self.generate_pose_candidates(
                                        self.workspace_p.object_geometries[req.object_idx].goal_pos)
        ###############################################################################################

        #################### select the right picking pose until it works #############################
        transfer_success = False
        for pose_id, placingPose in enumerate(placingPose_candidates):
            ####################### check the placing pose ###########################
            isPoseValid, FLAG, configToPlacingPose = self.planner_p.generateConfigBasedOnPose(
                    placingPose, currConfig, self.robot_p, self.workspace_p, req.armType)
            if not isPoseValid:
                print("This placing pose is not valid. Move on to next candidate.")
                continue
            else:
                ### check the connection with neighbors in the roadmap
                print("The placing pose works. Check its neighboring connections.")
                connectSuccess, placingPose_neighbors_idx, placingPose_neighbors_cost = self.planner_p.connectToNeighbors(
                            configToPlacingPose, self.robot_p, self.workspace_p, req.armType)
                if not connectSuccess:
                    print("This placing pose is not valid, due to no neighboring connections.")
                    print("Move on to next candidate.")
                    continue
                print("The placing pose is legitimate. Proceed to planning for placing.")
        
            ################### plan the path to placing configuration ###################
            placing_traj = self.planner_p.AstarPathFinding_new(currConfig, configToPlacingPose, 
                            pickingPose_neighbors_idx, pickingPose_neighbors_cost, 
                            placingPose_neighbors_idx, placingPose_neighbors_cost,
                            self.robot_p, self.workspace_p, req.armType)
            ### the planning has been finished, either success or failure
            if placing_traj != []:
                print("The transfer placing path for %s arm is successfully found" % req.armType)
                transfer_traj += placing_traj
                transfer_success = True
                break
            else:
                print("The transfer placing path for %s arm is not successfully found" % req.armType)
                print("Move on to next candidate")
                continue
            ############################################################################################
        
        if not transfer_success:
            print("No placing pose is qualified, either failed (1) placing pose (2) planning to placing")
            return RearrangeCylinderObjectResponse(False, object_path)

        ### Otherwise, congrats! Transfer is successful!
        ######################################### detach the object ###########################################
        ### Now we need to detach the object in hand before retracting the object (post-placing)
        self.planner_p.detachObject(self.workspace_p, self.robot_p, req.armType)
        #######################################################################################################
        ############# generate post-placing pose + cartesian move from placing to post-placing ################
        ### The arm leaves the object from ABOVE
        currConfig = self.getCurrentConfig(req.armType)
        postPlacingPose = copy.deepcopy(placingPose)
        postPlacingPose[0][2] += 0.05
        isPoseValid, FLAG, configToPostPlacingPose = self.planner_p.generateConfigBasedOnPose(
            postPlacingPose, currConfig, self.robot_p, self.workspace_p, req.armType)
        placeToPostPlaceTraj = self.planner_p.generateTrajectory_DirectConfigPath(
                currConfig, configToPostPlacingPose, self.robot_p, req.armType, self.workspace_p)
        finish_traj += placeToPostPlaceTraj
        ########################################################################################################
        
        ################################# prepare the path for the object ######################################
        ### get the current state
        currConfig = self.getCurrentConfig(req.armType)
        ### congrat! No problem of rearranging the current object
        ### prepare the object path
        object_path.transit_trajectory = self.generateArmTrajectory(
                                            transit_traj, req.armType, self.robot_p.motomanRJointNames)
        object_path.transfer_trajectory = self.generateArmTrajectory(
                                            transfer_traj, req.armType, self.robot_p.motomanRJointNames)
        object_path.finish_trajectory = self.generateArmTrajectory(
                                            finish_traj, req.armType, self.robot_p.motomanRJointNames)
        object_path.object_idx = req.object_idx
        return RearrangeCylinderObjectResponse(True, object_path)
        ########################################################################################################


    def generateOrientations(self, 
            default_orientation=np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]),
            possible_angles=[0, -15, -45, -30, 15, 45, 30]):
        ###### this function generates all orientation given default and angle options ######
        ### defalut_orientation: np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
        orientations = []
        for angle in possible_angles:
            angle *= math.pi / 180.0
            rotation = np.array([[math.cos(angle), -math.sin(angle), 0], 
                                 [math.sin(angle), math.cos(angle), 0], 
                                 [0, 0, 1]])
            orientation = np.dot(default_orientation, rotation)
            temp_quat = utils.getQuaternionFromRotationMatrix(orientation)
            orientations.append(temp_quat)
        return orientations

    
    def calculateReachabilityMap(self, orientation, placeholder_shape="cylinder"):
        ### calculate the reachability map for a particular orientation
        ### mark with the shape of different color specified
        ### Input: orientation is a format of quaternion (x,y,z,w)
        ### set up the two dictionaries as the mapping
        flag_color = {
            0: [0,1,0,1],
            1: [1,0,0,1],
            2: [0,0,1,1],
            3: [1,1,0,1]
        }
        obj_idx = 0
        for object_pos in self.workspace_p.all_goal_positions.values():
            print("current obj_idx: {}".format(obj_idx))
            temp_pos = copy.deepcopy(object_pos)
            temp_pos[2] = object_pos[2] + self.workspace_p.cylinder_height/2 - 0.03
            target_pose = [temp_pos, orientation]
            ### first check the pose
            isPoseValid, FLAG, configToPickingPose = self.planner_p.generateConfigBasedOnPose(
                        target_pose, obj_idx, self.robot_p, self.workspace_p, "Right_torso")
            if not isPoseValid:
                print("the pose is not valid")
                if placeholder_shape == "cylinder":
                    temp_placeholder_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                        radius=self.workspace_p.cylinder_radius, length=self.workspace_p.cylinder_height, 
                        rgbaColor=flag_color[FLAG], physicsClientId=self.planningClientID)
                if placeholder_shape == "sphere":
                    temp_placeholder_v = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                        radius=0.005, rgbaColor=flag_color[FLAG], physicsClientId=self.planningClientID)
                temp_placeholderM = p.createMultiBody(
                    baseVisualShapeIndex=temp_placeholder_v, basePosition=object_pos, physicsClientId=self.planningClientID)
            else:
                ### then check the pre-grasp pose
                isPoseValid, FLAG, prePickingPose, configToPrePickingPose = \
                    self.planner_p.generatePrePickingPose(
                        target_pose, obj_idx, self.robot_p, self.workspace_p, "Right_torso")
                if not isPoseValid:
                    print("the pre-picking pose is not valid")
                    if placeholder_shape == "cylinder":
                        temp_placeholder_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                            radius=self.workspace_p.cylinder_radius, length=self.workspace_p.cylinder_height, 
                            rgbaColor=flag_color[FLAG], physicsClientId=self.planningClientID)
                    if placeholder_shape == "sphere":
                        temp_placeholder_v = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                            radius=0.005, rgbaColor=flag_color[FLAG], physicsClientId=self.planningClientID)
                    temp_placeholderM = p.createMultiBody(
                        baseVisualShapeIndex=temp_placeholder_v, basePosition=object_pos, physicsClientId=self.planningClientID)
            if isPoseValid:
                print("both picking and pre-picking poses are valid")
                if placeholder_shape == "cylinder":
                    temp_placeholder_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                        radius=self.workspace_p.cylinder_radius, length=self.workspace_p.cylinder_height, 
                        rgbaColor=flag_color[FLAG], physicsClientId=self.planningClientID)
                if placeholder_shape == "sphere":
                    temp_placeholder_v = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                        radius=0.005, rgbaColor=flag_color[FLAG], physicsClientId=self.planningClientID)
                temp_placeholderM = p.createMultiBody(
                    baseVisualShapeIndex=temp_placeholder_v, basePosition=object_pos, physicsClientId=self.planningClientID)
            print("finish the object {}".format(obj_idx))
            obj_idx += 1
            input("press to continue")
        ### out of the loop
        self.robot_p.resetArmConfig_torso(
            self.robot_p.leftArmHomeConfiguration+self.robot_p.rightArmHomeConfiguration, self.robot_p.torsoHomeConfiguration)
        time.sleep(10000)


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

        while not rospy.has_param('/object_goal_deployment/discretization_x'):
            rospy.sleep(0.2)
        discretization_x = rospy.get_param('/object_goal_deployment/discretization_x')

        while not rospy.has_param('/object_goal_deployment/discretization_y'):
            rospy.sleep(0.2)
        discretization_y = rospy.get_param('/object_goal_deployment/discretization_y')

        while not rospy.has_param('/object_goal_deployment/object_interval_x'):
            rospy.sleep(0.2)
        object_interval_x = rospy.get_param('/object_goal_deployment/object_interval_x')

        while not rospy.has_param('/object_goal_deployment/object_interval_y'):
            rospy.sleep(0.2)
        object_interval_y = rospy.get_param('/object_goal_deployment/object_interval_y')

        while not rospy.has_param('/object_goal_deployment/side_clearance_x'):
            rospy.sleep(0.2)
        side_clearance_x = rospy.get_param('/object_goal_deployment/side_clearance_x')

        while not rospy.has_param('/object_goal_deployment/side_clearance_y'):
            rospy.sleep(0.2)
        side_clearance_y = rospy.get_param('/object_goal_deployment/side_clearance_y')

        while not rospy.has_param('/constrained_area/ceiling_height'):
            rospy.sleep(0.2)
        ceiling_height = rospy.get_param('/constrained_area/ceiling_height')

        while not rospy.has_param('/constrained_area/thickness_flank'):
            rospy.sleep(0.2)
        thickness_flank = rospy.get_param('/constrained_area/thickness_flank')

        while not rospy.has_param('/object_mesh_to_drop_in_real_scene/object_mesh_path'):
            rospy.sleep(0.2)
        object_mesh_path = rospy.get_param('/object_mesh_to_drop_in_real_scene/object_mesh_path')

        return basePosition, baseOrientation, urdfFile, \
            leftArmHomeConfiguration, rightArmHomeConfiguration, torsoHomeConfiguration, \
            standingBase_dim, table_dim, table_offset_x, \
            cylinder_radius, cylinder_height, \
            discretization_x, discretization_y, \
            object_interval_x, object_interval_y, \
            side_clearance_x, side_clearance_y, \
            ceiling_height, thickness_flank, \
            object_mesh_path


def main(args):
    pybullet_plan_scene = PybulletPlanScene(args)
    pybullet_plan_scene.planner_p.loadSamples()

    pybullet_plan_scene.rosInit()
    rate = rospy.Rate(10) ### 10hz

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
