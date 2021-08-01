#!/usr/bin/env python
from __future__ import division
from __future__ import print_function
import pybullet as p
import pybullet_data

import os
import random
import shutil
import math
import numpy as np
from scipy import spatial
import time
import IPython
import subprocess
from operator import itemgetter
import copy

import utils
from CollisionChecker import CollisionChecker

import rospy
from rospkg import RosPack
from pybullet_motoman.srv import AstarPathFinding, AstarPathFindingRequest
from pybullet_motoman.msg import Edge

class Planner(object):
    def __init__(self, rosPackagePath, server,
        isObjectInLeftHand=False, isObjectInRightHand=False,
        objectInLeftHand=None, objectInRightHand=None):
        self.planningServer = server
        self.rosPackagePath = rosPackagePath
        self.roadmapFolder = os.path.join(self.rosPackagePath, "roadmaps")
        self.collisionAgent_p = CollisionChecker(self.planningServer)
        self.nodes = {}
        self.nodes["Right_torso"] = []
        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand
        self.objectInLeftHand = objectInLeftHand
        self.objectInRightHand = objectInRightHand
        self.leftLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]
        self.rightLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]
        self.query_idx = 1 ### record the current planning query index
        self.loadIKdataset()


    def setRobotToConfig(self, ik_config, robot, armType):
        ### this function set the robot to certain config (with stepSimulation)
        ### this function must be called in the following situation
        ### (1) right before the collision check
        ### (2) attempt to get the pose of the end effector
        if armType == "Left_torso" or armType == "Right_torso":
            robot.setSingleArmToConfig_torso(ik_config[1:8], ik_config[0], armType)
        else:
            robot.setSingleArmToConfig(ik_config, armType)

    def loadIKdataset(self):
        armType = "Right_torso"
        self.IK_dataset_Right_torso = []
        ikdatasetFile = self.roadmapFolder + "/ik_dataset_" + str(armType) + ".txt"
        f_ikdataset = open(ikdatasetFile, "r")
        for line in f_ikdataset:
            line = line.split()
            line = [float(e) for e in line]
            self.IK_dataset_Right_torso.append(line)
        f_ikdataset.close()

    def loadSamples(self):
        arms = ["Right_torso"]
        ############### load the samples ###############
        for armType in arms:
            samplesFile = self.roadmapFolder + "/samples_" + str(armType) + ".txt"
            f_samples = open(samplesFile, "r")
            for line in f_samples:
                line = line.split()
                line = [float(e) for e in line[1:]]
                self.nodes[armType].append(line)
            f_samples.close()
        ################################################
        self.nsamples = len(self.nodes["Right_torso"])
        ### specify the needed parameters
        self.neighbors_const = 3.5 * math.e * (1 + 1.0/8)
        ### use k_n to decide the number of neighbors: #neighbors = k_n * log(#samples)
        self.num_neighbors = int(self.neighbors_const * math.log(self.nsamples))
        if self.num_neighbors > self.nsamples:
            self.num_neighbors = self.nsamples
        print("nsamples: ", self.nsamples)
        print("num_neighbors: ", self.num_neighbors)

    def generateSamples(self, nsamples, robot, workspace):
        if not os.path.exists(self.roadmapFolder):
            os.makedirs(self.roadmapFolder)
        self.nsamples = nsamples
        ### specify the needed parameters
        self.neighbors_const = 3.5 * math.e * (1 + 1.0/8)
        ### use k_n to decide the number of neighbors: #neighbors = k_n * log(#samples)
        self.num_neighbors = int(self.neighbors_const * math.log(self.nsamples))
        if self.num_neighbors > self.nsamples:
            self.num_neighbors = self.nsamples
        print("nsamples: ", self.nsamples)
        print("num_neighbors: ", self.num_neighbors)
        ################ sampling ################
        arms = ["Right_torso"]
        for armType in arms:
            samplesFile = self.roadmapFolder + "/samples_" + str(armType) + ".txt"
            self.samplingNodes(robot, workspace, armType)
            self.saveSamplesToFile(samplesFile, armType)
        ##########################################

    def samplingNodes(self, robot, workspace, armType):
        temp_counter = 0
        ### Let's start
        while temp_counter < self.nsamples:
            ### sample an IK configuration
            ikSolution = self.singleSampling_CSpace(robot, armType) ### this is for a single arm
            ########## move the robot to that configuration and then check ##########
            self.setRobotToConfig(ikSolution, robot, armType)
            isIKFallIntoRightRegion = self.sampleRegionCheck(robot, workspace, armType)
            if not isIKFallIntoRightRegion:
                continue
            isValid, FLAG = self.checkConfig_CollisionWithRobotAndKnownGEO(robot, workspace)
            if isValid:
                self.nodes[armType].append(ikSolution)
                temp_counter += 1
                print("finish the %s node" % str(temp_counter))
            ##########################################################################

    def singleSampling_CSpace(self, robot, armType):
        if armType == "Left" or "Left_torso":
            first_joint_index = 1
        if armType == "Right" or "Right_torso":
            first_joint_index = 8
        nArmJoints = int(len(robot.rightArmHomeConfiguration))
        ikSolution = []
        ### first consider if you have torso to handle
        if armType == "Left_torso" or armType == "Right_torso":
            ikSolution.append(random.uniform(robot.ll[0], robot.ul[0]))
        ### Now handle the arm
        for i in range(nArmJoints):
            ikSolution.append(
                random.uniform(robot.ll[first_joint_index+i], robot.ul[first_joint_index+i]))

        return ikSolution

    def sampleRegionCheck(self, robot, workspace, armType):
        if armType == "Left" or armType == "Left_torso":
            ee_idx = robot.left_ee_idx
        else:
            ee_idx = robot.right_ee_idx

        isIKFallIntoRightRegion = False
        pos_quat = p.getLinkState(robot.motomanGEO, ee_idx, physicsClientId=self.planningServer)
        pos = list(pos_quat[0])
        if (pos[2] > workspace.tablePosition[2] + workspace.table_dim[2] / 2 + workspace.ceiling_height):
            return isIKFallIntoRightRegion
        # if (pos[0] > workspace.constrained_area_x_limit[0] and pos[0] < workspace.constrained_area_x_limit[1]) and \
        #    (pos[1] > workspace.constrained_area_y_limit[0] and pos[1] < workspace.constrained_area_y_limit[1]):
        #     if (pos[2] < workspace.tablePosition[2] + workspace.table_dim[2] / 2 + workspace.cylinder_height / 2 + 0.01):
        #         return isIKFallIntoRightRegion
        return True

    def saveSamplesToFile(self, samplesFile, armType):
        f_samples = open(samplesFile, "w")
        for node_idx in range(len(self.nodes[armType])):
            node = self.nodes[armType][node_idx]
            f_samples.write(str(node_idx))
            for k in range(len(node)):
                f_samples.write(" " + str(node[k]))
            f_samples.write("\n")
        f_samples.close()

    def samplesConnect(self, robot, workspace, armType):
        connectivity = np.zeros((self.nsamples, self.nsamples))
        tree = spatial.KDTree(self.nodes[armType]) ### use KD tree to arrange neighbors assignment
        connectionsFile = self.roadmapFolder + "/connections_" + str(armType) + ".txt"
        f_connection = open(connectionsFile, "w")
        ### for each node
        for node_idx in range(len(self.nodes[armType])):
            queryNode = self.nodes[armType][node_idx]
            knn = tree.query(queryNode, k=self.num_neighbors, p=2)

            neighbors_connected = 0
            ### for each potential neighbor
            for j in range(len(knn[1])):
                ### first check if this query node has already connected to enough neighbors
                if neighbors_connected >= self.num_neighbors:
                    break
                if knn[1][j] == node_idx:
                    ### if the neighbor is the query node itself
                    continue
                if connectivity[node_idx][knn[1][j]] == 1:
                    ### the connectivity has been checked before
                    neighbors_connected += 1
                    continue
                ### Otherwise, check the edge validity
                ### in terms of collision with the robot itself and all known geometries (e.g. table/shelf)
                ### between the query node and the current neighbor
                neighbor = self.nodes[armType][knn[1][j]]
                isEdgeValid, FLAG = self.checkEdgeValidity_knownGEO(queryNode, neighbor, robot, workspace, armType)
                if isEdgeValid:
                    ### write this edge information with their costs and labels into the txt file
                    f_connection.write(str(node_idx) + " " + str(knn[1][j]) + " " + str(knn[0][j]) + "\n")
                    connectivity[node_idx][knn[1][j]] = 1
                    connectivity[knn[1][j]][node_idx] = 1
                    neighbors_connected += 1
            print("Number of neighbors for current node " + str(node_idx) + ": " + str(neighbors_connected))
        f_connection.close()


    def updateMeshBasedonLocalPose(self, object_idx, robot, workspace, armType):
        ### the planning version of the API "updateRealObjectBasedonLocalPose" in executor
        if armType == "Left" or armType == "Left_torso":
            # ee_idx = robot.left_ee_idx
            # objectInHand = self.objectInLeftHand
            # curr_ee_pose = robot.left_ee_pose
            # object_global_pose = self.getObjectGlobalPose(self.leftLocalPose, curr_ee_pose)
            object_global_pose = self.getObjectGlobalPose(self.leftLocalPose, robot.left_ee_pose)
            p.resetBasePositionAndOrientation(
                self.objectInLeftHand, object_global_pose[0], object_global_pose[1], 
                physicsClientId=self.planningServer)
            ### update the position and orientation of the object manipulated (ignore the orientation at this point)
            workspace.object_geometries[object_idx].curr_pos = object_global_pose[0]
        if armType == "Right" or armType == "Right_torso":
            # ee_idx = robot.right_ee_idx
            # objectInHand = self.objectInRightHand
            # curr_ee_pose = robot.right_ee_pose
            # object_global_pose = self.getObjectGlobalPose(self.rightLocalPose, curr_ee_pose)
            object_global_pose = self.getObjectGlobalPose(self.rightLocalPose, robot.right_ee_pose)
            p.resetBasePositionAndOrientation(
                self.objectInRightHand, object_global_pose[0], object_global_pose[1], 
                physicsClientId=self.planningServer)
            ### update the position and orientation of the object manipulated (ignore the orientation at this point)
            workspace.object_geometries[object_idx].curr_pos = object_global_pose[0]            

    def getObjectGlobalPose(self, local_pose, ee_global_pose):
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0], ee_global_pose[1],
            local_pose[0], local_pose[1])
        object_global_pose = [list(temp_object_global_pose[0]), list(temp_object_global_pose[1])]

        return object_global_pose 


    def randomRestPose(self, robot, armType):
        ### option 1: choose from IK dataset
        if armType == "Right_torso":
            temp_idx = random.randint(0, len(self.IK_dataset_Right_torso)-1)
            right_torso_rp = self.IK_dataset_Right_torso[temp_idx]
            rp = [right_torso_rp[0]] + robot.leftArmCurrConfiguration + right_torso_rp[1:8] + robot.rightHandCurrConfiguration
            return rp

        ### option 2
        # rp = []
        # for i_joint in range(len(robot.rp)):
        #     joint_value = random.uniform(robot.ll[i_joint], robot.ul[i_joint])
        #     rp.append(joint_value)
        # return rp

        ### option 3
        # rp = []
        # for i_joint in range(len(robot.rp)):
        #     if armType == 'Left_torso' or armType == "Right_torso":
        #         if i_joint == 0: 
        #             rp.append(robot.rp[i_joint])
        #             continue
        #     joint_value = robot.rp[i_joint] + random.uniform(-0.34, 0.34)
        #     ### clipping
        #     if joint_value < robot.ll[i_joint]:
        #         joint_value = robot.ll[i_joint] + 0.017
        #     if joint_value > robot.ul[i_joint]:
        #         joint_value = robot.ul[i_joint] - 0.017
        #     rp.append(joint_value)
        # return rp

    def generatePrePickingOrPostPlacingPose(self, pose, rest_config, object_idx, robot, workspace, armType):
        ### This function generates a pre-picking / post-placing pose based on grasp pose
        ### It is 5cm (0.05m) behind the approaching direction of z axis (local axis of the end effector)
        ### Input: pose: [[x,y,z], [x,y,z,w]]
        if armType == "Left" or armType == "Left_torso":
            ee_idx = robot.left_ee_idx
            first_joint_index = 1
        if armType == "Right" or armType == "Right_torso":
            ee_idx = robot.right_ee_idx
            first_joint_index = 8

        if armType == "Left":
            rest_pose = robot.torsoCurrConfiguration + rest_config + robot.rightArmCurrConfiguration + robot.rightHandCurrConfiguration
        if armType == "Right":
            rest_pose = robot.torsoCurrConfiguration + robot.leftArmCurrConfiguration + rest_config + robot.rightHandCurrConfiguration
        if armType == "Left_torso":
            rest_pose = [rest_config[0]] + rest_config[1:8] + robot.rightArmCurrConfiguration + robot.rightHandCurrConfiguration
        if armType == "Right_torso":
            rest_pose = [rest_config[0]] + robot.leftArmCurrConfiguration + rest_config[1:8] + robot.rightHandCurrConfiguration

        temp_rot_matrix = p.getMatrixFromQuaternion(pose[1])
        ### local z-axis of the end effector
        temp_approaching_direction = [temp_rot_matrix[2], temp_rot_matrix[5], temp_rot_matrix[8]]
        temp_pos = list(np.array(pose[0]) - 0.05*np.array(temp_approaching_direction))
        new_pose = [temp_pos, pose[1]] ### the quaternion remains the same as input pose

        ### check the IK of the pose
        q_newPoseIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO, 
                                endEffectorLinkIndex=ee_idx, 
                                targetPosition=new_pose[0], 
                                targetOrientation=new_pose[1], 
                                lowerLimits=robot.ll, upperLimits=robot.ul, 
                                jointRanges=robot.jr, restPoses=rest_pose,
                                maxNumIterations=2000, residualThreshold=0.0000001,
                                physicsClientId=robot.server)
        if armType == "Left" or armType == "Right":
            singleArmConfig_IK = list(q_newPoseIK[first_joint_index:first_joint_index+7])
        if armType == "Left_torso" or armType == "Right_torso":
            singleArmConfig_IK = [q_newPoseIK[0]] + list(q_newPoseIK[first_joint_index:first_joint_index+7])
        ########## move the robot to that configuration and then check ##########
        self.setRobotToConfig(singleArmConfig_IK, robot, armType)
        isIKValid, FLAG = self.checkPoseIK(new_pose, object_idx, robot, workspace, armType)
        ##########################################################################

        trials = 0
        while (not isIKValid) and (trials < 5):
            ### try another IK (not specify rest pose)
            ### try another IK given curernt rest pose with random noise
            rest_pose = self.randomRestPose(robot, armType)
            q_newPoseIK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=new_pose[0],
                                    targetOrientation=new_pose[1],
                                    lowerLimits=robot.ll, upperLimits=robot.ul, 
                                    jointRanges=robot.jr, restPoses=rest_pose,
                                    maxNumIterations=2000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            if armType == "Left" or armType == "Right":
                singleArmConfig_IK = list(q_newPoseIK[first_joint_index:first_joint_index+7])
            if armType == "Left_torso" or armType == "Right_torso":
                singleArmConfig_IK = [q_newPoseIK[0]] + list(q_newPoseIK[first_joint_index:first_joint_index+7])
            ########## move the robot to that configuration and then check ##########
            self.setRobotToConfig(singleArmConfig_IK, robot, armType)
            isIKValid, FLAG = self.checkPoseIK(new_pose, object_idx, robot, workspace, armType)
            if isIKValid: break
            ### otherwise
            trials += 1
            ##########################################################################
        ### you need to return both the statement whether the pose is valid
        ### and the valid configuration the pose corresponds to
        return isIKValid, FLAG, new_pose, singleArmConfig_IK


    def generateConfigBasedOnPose(self, pose, rest_config, object_idx, robot, workspace, armType):
        ### This function checks the validity of a pose by checking its corresponding config (IK)
        ### Input: pose: [[x,y,z],[x,y,z,w]]
        ###        armType: "Left" or "Right"
        ### Output: isIKValid (bool), singleArmConfig_IK (list)), FLAG
        if armType == "Left" or armType == "Left_torso":
            ee_idx = robot.left_ee_idx
            first_joint_index = 1
        if armType == "Right" or armType == "Right_torso":
            ee_idx = robot.right_ee_idx
            first_joint_index = 8
        
        if armType == "Left":
            rest_pose = robot.torsoCurrConfiguration + rest_config + robot.rightArmCurrConfiguration + robot.rightHandCurrConfiguration
        if armType == "Right":
            rest_pose = robot.torsoCurrConfiguration + robot.leftArmCurrConfiguration + rest_config + robot.rightHandCurrConfiguration
        if armType == "Left_torso":
            rest_pose = [rest_config[0]] + rest_config[1:8] + robot.rightArmCurrConfiguration + robot.rightHandCurrConfiguration
        if armType == "Right_torso":
            rest_pose = [rest_config[0]] + robot.leftArmCurrConfiguration + rest_config[1:8] + robot.rightHandCurrConfiguration

        ### we add rest pose in the IK solver to get as high-quality IK as possible
        config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                endEffectorLinkIndex=ee_idx,
                                targetPosition=pose[0],
                                targetOrientation=pose[1],
                                lowerLimits=robot.ll, upperLimits=robot.ul, 
                                jointRanges=robot.jr, restPoses=rest_pose,
                                maxNumIterations=2000, residualThreshold=0.0000001,
                                physicsClientId=robot.server)
        if armType == "Left" or armType == "Right":
            singleArmConfig_IK = list(config_IK[first_joint_index:first_joint_index+7])
        if armType == "Left_torso" or armType == "Right_torso":
            singleArmConfig_IK = [config_IK[0]] + list(config_IK[first_joint_index:first_joint_index+7])
        ########## move the robot to that configuration and then check ##########
        self.setRobotToConfig(singleArmConfig_IK, robot, armType)
        isIKValid, FLAG = self.checkPoseIK(pose, object_idx, robot, workspace, armType)
        ##########################################################################
        
        trials = 0
        while (not isIKValid) and (trials < 5):
            ### try another IK (not specify rest pose)
            ### try another IK given curernt rest pose with random noise
            rest_pose = self.randomRestPose(robot, armType)
            config_IK = p.calculateInverseKinematics(bodyUniqueId=robot.motomanGEO,
                                    endEffectorLinkIndex=ee_idx,
                                    targetPosition=pose[0],
                                    targetOrientation=pose[1],
                                    lowerLimits=robot.ll, upperLimits=robot.ul, 
                                    jointRanges=robot.jr, restPoses=rest_pose,
                                    maxNumIterations=2000, residualThreshold=0.0000001,
                                    physicsClientId=robot.server)
            if armType == "Left" or armType == "Right":
                singleArmConfig_IK = list(config_IK[first_joint_index:first_joint_index+7])
            if armType == "Left_torso" or armType == "Right_torso":
                singleArmConfig_IK = [config_IK[0]] + list(config_IK[first_joint_index:first_joint_index+7])
            ########## move the robot to that configuration and then check ##########
            self.setRobotToConfig(singleArmConfig_IK, robot, armType)
            isIKValid, FLAG = self.checkPoseIK(pose, object_idx, robot, workspace, armType)
            if isIKValid: break
            ### otherwise
            trials += 1
            ##########################################################################
        ### you need to return both the statement whether the pose is valid
        ### and the valid configuration the pose corresponds to
        return isIKValid, FLAG, singleArmConfig_IK

    def checkPoseIK(self, desired_ee_pose, object_idx, robot, workspace, armType):
        ######## before calling this function, don't forget to call API: setRobotToConfig ########
        ### This function checks if an IK solution is valid in terms of
        ### (1) small error from the desired pose (position error + quaternion error)
        ### (2) no collision occurred (all types of collisions)

        ### Input: desired_ee_pose: [[x,y,z], [x,y,z,w]]
        ###        armType: "Left" or "Right"
        ### Output: isConfigValid (bool) indicating whether the IK is valid
        ###         FLAG (int): indicating the validity/invalidity reason
        ###                     0: no any issues (pass all IK check)
        #                       1: reachability
        #                       2: robot self-collision
        #                       3: collision between robot and static geometry
        #                       4: collision between robot and objects in the scene
        #                       5: moving object collides with static geometry
        #                       6: moving object collides with other objects in the scene
        
        isConfigValid, FLAG = self.checkConfig_reachability(desired_ee_pose, robot, armType)
        if not isConfigValid: 
            return isConfigValid, FLAG
        ### Congrats! The IK successfully passed reachability checker. 
        ### Then check if there is collision
        isConfigValid, FLAG = self.checkConfig_AllCollisions(object_idx, robot, workspace, armType)
        return isConfigValid, FLAG


    def checkConfig_reachability(self, desired_ee_pose, robot, armType):
        ######## before calling this function, don't forget to call API: setRobotToConfig ########
        ### This function purely checks if an IK solution is reachable
        ### small position and orientation error
        if armType == "Left" or armType == "Left_torso":
            actual_ee_pose = copy.deepcopy(robot.left_ee_pose)
        if armType == "Right" or armType == "Right_torso":
            actual_ee_pose = copy.deepcopy(robot.right_ee_pose)
        ### if the ee_idx is within 2.0cm(0.02m) Euclidean distance from the desired one, we accept it
        ee_dist_pos = utils.computePoseDist_pos(actual_ee_pose[0], desired_ee_pose[0])
        # print("actual_pose: " + str(actual_ee_pose[0]))
        # print("desired_pose: " + str(desired_ee_pose))
        # print("position error: " + str(ee_dist_pos))
        if ee_dist_pos > 0.02:
            print("IK not reachable as position error exceeds 2cm: " + str(ee_dist_pos))
            ### raise the flag
            isConfigValid = False
            FLAG = 1
            return isConfigValid, FLAG
        else:
            ### Now check orientation error
            ee_dist_quat = utils.computePoseDist_quat(actual_ee_pose[1], desired_ee_pose[1])
            if ee_dist_quat > 0.8:
                print("IK not reachable as quaternion error exceeds 0.8: " + str(ee_dist_quat))
                ### raise the flag
                isConfigValid = False
                FLAG = 1
                return isConfigValid, FLAG
        ### otherwise, this config (IK) is reachable
        isConfigValid = True
        FLAG = 0
        return isConfigValid, FLAG

    
    def checkConfig_AllCollisions(self, object_idx, robot, workspace, armType):
        ######## before calling this function, don't forget to call API: setRobotToConfig ########
        ### This function checks all collisions
        ### Common: no robot self collision and collsions between robot and knownGEO AT ALL TIME
        ### no other collisions
        ###     (i) no object in hand (e.g., "transit"): no collision between the robot and the static object
        ###     (ii) object in hand (e.g., "transfer"): 
        ###           no collision between the robot and the moving object
        ###           no collision between the moving object and knownGEO

        ############  first check potential collision between robot and knownGEO & robot ############
        isConfigValid, FLAG = self.checkConfig_CollisionWithRobotAndKnownGEO(robot, workspace)
        if isConfigValid == False: 
            # print("the robot either collides with itself or with known GEO")
            return isConfigValid, FLAG
        #############################################################################################

        ################# then check potential collision between with all objects ###################
        isConfigValid, FLAG = self.checkConfig_ObjectCollisions(
                                            object_idx, robot, workspace, armType)
        #############################################################################################

        return isConfigValid, FLAG

    
    def checkConfig_ObjectCollisions(self, object_idx, robot, workspace, armType):
        ######## before calling this function, don't forget to call API: setRobotToConfig ########
        ### This function checks all collisions related to objects

        ########## If currently it is in hand manipulation, also move the object ####################
        if (self.isObjectInLeftHand and (armType == "Left" or armType == "Left_torso")) or \
                    (self.isObjectInRightHand and (armType == "Right" or armType == "Right_torso")):
            self.updateMeshBasedonLocalPose(object_idx, robot, workspace, armType)
        #############################################################################################

        ############## then check potential collision between robot and objects #####################
        isConfigValid, FLAG = self.checkConfig_CollisionBetweenRobotAndObjects(object_idx, robot, workspace)
        if isConfigValid == False:
            # print("the robot arm collide with any of the objects (static or manipulated)")
            return isConfigValid, FLAG
        #############################################################################################

        ########################## If it is in-hand manipulation, ###################################
        ## then also check potential collision between the moving object and other static objects ###
        if self.isObjectInLeftHand or self.isObjectInRightHand:
            isConfigValid, FLAG = self.checkConfig_CollisionWithMovingObject(object_idx, robot, workspace, armType)
            if isConfigValid == False:
                # print("the manipulated object collides with knownGEO or remaining static object")
                return isConfigValid, FLAG
        #############################################################################################

        return isConfigValid, FLAG


    def checkConfig_CollisionWithRobotAndKnownGEO(self, robot, workspace):
        ######## before calling this function, don't forget to call API: setRobotToConfig ########

        ### check if there is collision
        if self.collisionAgent_p.collisionCheck_selfCollision(robot.motomanGEO) == True:
            # print("robot self collision")
            ### raise the flag
            isConfigValid = False
            FLAG = 2
            return isConfigValid, FLAG

        if self.collisionAgent_p.collisionCheck_robot_knownGEO(
                    robot.motomanGEO, workspace.known_geometries) == True:
            # print("robot collide with known geometries")
            ### raise the flag
            isConfigValid = False
            FLAG = 3
            return isConfigValid, FLAG

        ### If you reach here, the configuration passes collision check with known geometry
        # print("pass collision checker with known GEO")
        isConfigValid = True
        FLAG = 0
        return isConfigValid, FLAG


    def checkConfig_CollisionBetweenRobotAndObjects(self, object_idx, robot, workspace):
        ######## before calling this function, don't forget to call API: setRobotToConfig ########
        ### check if there is collision between the robot and the object
        ### make object_geometries a dictionary
        object_geometries = {obj_info.object_index : obj_info.geo for obj_info in workspace.object_geometries.values()}
        # object_geometries = [obj_info.geo for obj_info in workspace.object_geometries.values()]
        if self.collisionAgent_p.collisionCheck_robot_objectGEO(
            robot.motomanGEO, object_geometries, object_idx, 
            self.isObjectInLeftHand, self.isObjectInRightHand) == True:
            # print("collision between robot and the objects")
            ### raise the flag
            isConfigValid = False
            FLAG = 4
            return isConfigValid, FLAG
        
        ### If you reach here, the configuration passes collision check with object geometry
        # print("pass IK collision checker between the robot and object geometry")
        isConfigValid = True
        FLAG = 0
        return isConfigValid, FLAG


    def checkConfig_CollisionWithMovingObject(self, object_idx, robot, workspace, armType):
        ######## before calling this function, don't forget to call API: setRobotToConfig ########
        ### we need to check if there is any collision between the manipulated object and
        ### (1) knownGEO
        ### (2) remaining object not manipulated

        ### (1) check collision between manipulated object and knownGEO 
        if armType == "Left" or armType == "Left_torso":
            manipulated_object_geometry = self.objectInLeftHand
        if armType == "Right" or armType == "Right_torso":
            manipulated_object_geometry = self.objectInRightHand
        if self.collisionAgent_p.collisionCheck_object_knownGEO(
                            manipulated_object_geometry, workspace.known_geometries) == True:
            # print("moving object collide with known geomtries")
            ### raise the flag
            isConfigValid = False
            FLAG = 5
            return isConfigValid, FLAG

        ### congrats! But now we still need to check collision
        ### (2) between object_geometry and other objects
        ### make static_object_geometries a dictionary
        static_object_geometries = {
            obj_info.object_index : obj_info.geo for obj_info in workspace.object_geometries.values() if obj_info.object_index != object_idx}
        # static_object_geometries = [
        #     obj_info.geo for obj_info in workspace.object_geometries.values() if obj_info.object_index != object_idx]
        if self.collisionAgent_p.collisionCheck_object_objectGEO(
                                        manipulated_object_geometry, static_object_geometries) == True:
            # print("moving object collide with remaining object geometries")
            ### raise the flag
            isConfigValid = False
            FLAG = 6
            return isConfigValid, FLAG
        
        ### if you reach here, the configuration passes collision check with moving object geometry
        isConfigValid = True
        FLAG = 0
        return isConfigValid, FLAG


    def checkEdgeValidity_DirectConfigPath(self, n1, n2, object_idx, robot, workspace, armType):
        ### Input: n1, n2: node (a list of 7 or 8 joint values)
        ### Output: bool value indicates whether the transition from n1 to n2 is valid 
        ###         and a flag which indicates the failure explanation

        # nseg = 5
        # min_degree = math.pi / 90
        min_degree = math.pi / 180 * 5 ### make it sparsely interpolated to speed up collision check

        if armType == "Left" or armType == "Right":
            nseg = int(max(
                abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
                abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
            if nseg == 0: nseg += 1
            # print("nseg: " + str(nseg))

            for i in range(0, nseg+1):
                interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
                interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
                interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
                interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
                interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
                interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
                interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
                intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
                ########## move the robot to that configuration and then check ##########
                self.setRobotToConfig(intermNode, robot, armType)
                ### check if there is collision
                isConfigValid, FLAG = self.checkConfig_AllCollisions(object_idx, robot, workspace, armType)
                if not isConfigValid:
                    isEdgeValid = False
                    return isEdgeValid, FLAG
                ##########################################################################
            ### Reach here because the edge pass the collision check
            isEdgeValid = True
            FLAG = 0
            return isEdgeValid, FLAG

        if armType == "Left_torso" or armType == "Right_torso":
            nseg = int(max(
                abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
                abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6]), abs(n1[7]-n2[7])) / min_degree)
            if nseg == 0: nseg += 1
            # print("nseg: " + str(nseg))            

            for i in range(0, nseg+1):
                # print(i)
                interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
                interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
                interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
                interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
                interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
                interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
                interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
                interm_j7 = n1[7] + (n2[7]-n1[7]) / nseg * i
                intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6, interm_j7]
                ########## move the robot to that configuration and then check ##########
                self.setRobotToConfig(intermNode, robot, armType)
                ### check if there is collision
                isConfigValid, FLAG = self.checkConfig_AllCollisions(object_idx, robot, workspace, armType)
                if not isConfigValid:
                    isEdgeValid = False
                    return isEdgeValid, FLAG
                ##########################################################################
            ### Reach here because the edge pass the collision check
            isEdgeValid = True
            FLAG = 0
            return isEdgeValid, FLAG

    def checkEdgeValidity_objectGEO(self, n1, n2, object_idx, robot, workspace, armType):
        ### Input: n1, n2: node (a list of 7 or 8 joint values)
        ### Output: bool value indicates whether the transition from n1 to n2 is valid
        ###         and a flag which indicates the failure explanation

        # nseg = 5
        # min_degree = math.pi / 90
        min_degree = math.pi / 180 * 5 ### make it sparsely interpolated to speed up collision check

        if armType == "Left" or armType == "Right":
            nseg = int(max(
                abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
                abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
            if nseg == 0: nseg += 1
            # print("nseg: " + str(nseg))

            for i in range(0, nseg+1):
                interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
                interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
                interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
                interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
                interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
                interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
                interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
                intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
                ########## move the robot to that configuration and then check ##########
                self.setRobotToConfig(intermNode, robot, armType)
                ### check if there is collision
                isConfigValid, FLAG = self.checkConfig_ObjectCollisions(object_idx, robot, workspace, armType)
                if not isConfigValid:
                    isEdgeValid = False
                    return isEdgeValid, FLAG
                ##########################################################################
            ### Reach here because the edge pass the collision check
            isEdgeValid = True
            FLAG = 0
            return isEdgeValid, FLAG

        if armType == "Left_torso" or armType == "Right_torso":
            nseg = int(max(
                abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
                abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6]), abs(n1[7]-n2[7])) / min_degree)
            if nseg == 0: nseg += 1
            # print("nseg: " + str(nseg))            

            for i in range(0, nseg+1):
                interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
                interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
                interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
                interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
                interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
                interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
                interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
                interm_j7 = n1[7] + (n2[7]-n1[7]) / nseg * i
                intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6, interm_j7]
                ########## move the robot to that configuration and then check ##########
                self.setRobotToConfig(intermNode, robot, armType)
                ### check if there is collision
                isConfigValid, FLAG = self.checkConfig_ObjectCollisions(object_idx, robot, workspace, armType)
                if not isConfigValid:
                    isEdgeValid = False
                    return isEdgeValid, FLAG
                ##########################################################################
            ### Reach here because the edge pass the collision check
            isEdgeValid = True
            FLAG = 0
            return isEdgeValid, FLAG

    def checkEdgeValidity_knownGEO(self, n1, n2, robot, workspace, armType):
        ### Input: n1, n2: node (a list of 7 or 8 joint values)
        ### Output: bool value indicates whether the transition from n1 to n2 is valid
        ###         and a flag which indicates the failure explanation

        # nseg = 5
        # min_degree = math.pi / 90
        min_degree = math.pi / 180 * 5 ### make it sparsely interpolated to speed up collision check

        if armType == "Left" or armType == "Right":
            nseg = int(max(
                abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
                abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
            if nseg == 0: nseg += 1
            # print("nseg: " + str(nseg))

            for i in range(0, nseg+1):
                interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
                interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
                interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
                interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
                interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
                interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
                interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
                intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
                ########## move the robot to that configuration and then check ##########
                self.setRobotToConfig(intermNode, robot, armType)
                ### check if there is collision
                isConfigValid, FLAG = self.checkConfig_CollisionWithRobotAndKnownGEO(robot, workspace)
                if not isConfigValid:
                    isEdgeValid = False
                    return isEdgeValid, FLAG
                ##########################################################################
            ### Reach here because the edge pass the collision check
            isEdgeValid = True
            FLAG = 0
            return isEdgeValid, FLAG

        if armType == "Left_torso" or armType == "Right_torso":
            nseg = int(max(
                abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
                abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6]), abs(n1[7]-n2[7])) / min_degree)
            if nseg == 0: nseg += 1

            for i in range(0, nseg+1):
                interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
                interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
                interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
                interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
                interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
                interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
                interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
                interm_j7 = n1[7] + (n2[7]-n1[7]) / nseg * i
                intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6, interm_j7]
                ########## move the robot to that configuration and then check ##########
                self.setRobotToConfig(intermNode, robot, armType)
                ### check if there is collision
                isConfigValid, FLAG = self.checkConfig_CollisionWithRobotAndKnownGEO(robot, workspace)
                if not isConfigValid:
                    isEdgeValid = False
                    return isEdgeValid, FLAG
                ##########################################################################
            ### Reach here because the edge pass the collision check
            isEdgeValid = True
            FLAG = 0
            return isEdgeValid, FLAG


    def generateTrajectory_DirectConfigPath(self, n1, n2, robot, armType, object_idx, workspace):
        ### This function generates a trajectory based on two configs (which has been proved to be valid transition)
        ### Input: n1, n2: node (a list of 7 or 8 joint values)
        ### output: an edge trajectory (config_edge_traj) which includes the endtail but not the head
        ###         format: a list of list(7 or 8 joint values)

        config_edge_traj = []
        # nseg = 5
        min_degree = math.pi / 180 * 1 ### want more waypoint to move more naturally

        if armType == "Left" or armType == "Right":
            nseg = int(max(
                abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
                abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6])) / min_degree)
            if nseg == 0: nseg += 1
            # print("nseg: " + str(nseg))
            ### we don't include the head (i=0)
            for i in range(1, nseg+1):
                interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
                interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
                interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
                interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
                interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
                interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
                interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
                intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
                robot.setSingleArmToConfig(intermNode, armType)
                ########## temporarily add here for visualization ##########
                if (self.isObjectInLeftHand and (armType == "Left" or armType == "Left_torso")) or \
                            (self.isObjectInRightHand and (armType == "Right" or armType == "Right_torso")):
                    self.updateMeshBasedonLocalPose(object_idx, robot, workspace, armType)
                ############################################################
                # time.sleep(0.1)
                config_edge_traj.append(intermNode)
        
        if armType == "Left_torso" or armType == "Right_torso":
            nseg = int(max(
                abs(n1[0]-n2[0]), abs(n1[1]-n2[1]), abs(n1[2]-n2[2]), abs(n1[3]-n2[3]),
                abs(n1[4]-n2[4]), abs(n1[5]-n2[5]), abs(n1[6]-n2[6]), abs(n1[7]-n2[7])) / min_degree)
            if nseg == 0: nseg += 1
            # print("nseg: " + str(nseg))            
            for i in range(1, nseg+1):
                interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
                interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
                interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
                interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
                interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
                interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
                interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
                interm_j7 = n1[7] + (n2[7]-n1[7]) / nseg * i
                intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6, interm_j7]
                robot.setSingleArmToConfig_torso(intermNode[1:8], intermNode[0], armType)
                ########## temporarily add here for visualization ##########
                if (self.isObjectInLeftHand and (armType == "Left" or armType == "Left_torso")) or \
                            (self.isObjectInRightHand and (armType == "Right" or armType == "Right_torso")):
                    self.updateMeshBasedonLocalPose(object_idx, robot, workspace, armType)
                ############################################################
                # time.sleep(0.1)
                config_edge_traj.append(intermNode)

        return config_edge_traj


    def serviceCall_astarPathFinding(self, 
            violated_edges, initialConfig, targetConfig, 
            start_neighbors_idx, goal_neighbors_idx, start_neighbors_cost, goal_neighbors_cost,
            robot, workspace, armType):
        ### violated_edges: [Edge(), Edge(), ...]
        ### prepare the astarPathFindingRequest
        rospy.wait_for_service("astar_path_finding")
        request = AstarPathFindingRequest()
        request.query_idx = self.query_idx
        request.start_idx = self.nsamples
        request.goal_idx = self.nsamples + 1
        request.start_config = initialConfig
        request.goal_config = targetConfig
        request.violated_edges = violated_edges
        request.armType = armType
        request.start_neighbors_idx = start_neighbors_idx
        request.goal_neighbors_idx = goal_neighbors_idx
        request.start_neighbors_cost = start_neighbors_cost
        request.goal_neighbors_cost = goal_neighbors_cost

        try:
            astarSearch = rospy.ServiceProxy("astar_path_finding", AstarPathFinding)
            response = astarSearch(request.query_idx, 
                request.start_idx, request.goal_idx,
                request.start_config, request.goal_config,
                request.start_neighbors_idx, request.goal_neighbors_idx,
                request.start_neighbors_cost, request.goal_neighbors_cost,
                request.violated_edges, request.armType)
            return response.searchSuccess, list(response.path)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    def AstarPathFinding(self, initialConfig, targetConfig, object_idx, robot, workspace, armType):
        ### Input: initialConfig, targetConfig [q1, q2, ..., q7]
        ### Output: traj (format: [joint_state1, joint_state2, ...])
        ###         and joint_state is a list of joint values
        ### first prepare the start_goal file

        result_traj = [] ### the output we want to construct
        isPathValid = False
        print("current planning query: ", self.query_idx)
        violated_edges = [] ### initially there are no violated edges
        ### find the neighbors for the start and the goal
        # start_time = time.time()
        start_neighbors_idx, goal_neighbors_idx, start_neighbors_cost, goal_neighbors_cost = \
            self.findNeighborsForStartAndGoal(
                initialConfig, targetConfig, object_idx, robot, workspace, armType)
        # print("Time for start and goal to connect to neighbors in the roadmap: {}".format(time.time() - start_time))
        
        counter = 0
        while (isPathValid == False):
            counter += 1
            ### trigger new call within the same query idx
            # start_time = time.time()
            searchSuccess, path =  self.serviceCall_astarPathFinding(
                    violated_edges, initialConfig, targetConfig, 
                    start_neighbors_idx, goal_neighbors_idx,
                    start_neighbors_cost, goal_neighbors_cost,
                    robot, workspace, armType)
            # print("Time for service call for astarPathFinding: {}".format(time.time() - start_time))
            if searchSuccess == False:
                print("the plan fails at the " + str(counter) + "th trial...")
                ### the plan fails, could not find a solution
                self.query_idx += 1
                return result_traj ### an empty trajectory
            ### otherwise, we need collision check and smoothing (len(path) >= 3)
            # start_time = time.time()
            smoothed_path, isPathValid, violated_edges = self.smoothPath(
                    path, initialConfig, targetConfig, object_idx, robot, workspace, armType)
            # print("Time for smooth the path: {}".format(time.time() - start_time))

        ### congrats, the path is valid and finally smoothed, let's generate trajectory
        print("smoothed path: ", smoothed_path)
        ### directly generate trajectory based on the new path
        for i in range(0, len(smoothed_path)-1):
            if i == 0:
                config1 = initialConfig
            else:
                config1 = self.nodes[armType][smoothed_path[i]]
            if i == (len(smoothed_path)-2):
                config2 = targetConfig
            else:
                config2 = self.nodes[armType][smoothed_path[i+1]]
            ### get edge trajectory
            config_edge_traj = self.generateTrajectory_DirectConfigPath(config1, config2, robot, armType, object_idx, workspace)
            # result_traj.append(config_edge_traj)
            result_traj += config_edge_traj

        ### before you claim the victory of this query, increment the planning query
        ### so as to tell people this query is over, next time is a new query
        self.query_idx += 1

        return result_traj


    def AstarPathFinding_new(self, initialConfig, targetConfig,
                start_neighbors_idx, start_neighbors_cost, 
                goal_neighbors_idx, goal_neighbors_cost,
                object_idx, robot, workspace, armType):
        ### Input: initialConfig, targetConfig [q1, q2, ..., q7]
        ###        neighbors_idx as well as neighbors_cost should not be empty when entering in this function
        ### Output: traj (format: [joint_state1, joint_state2, ...])
        ###         and joint_state is a list of joint values

        # print("===================================")
        # print("===================================")
        # print("start_neighbors_idx: " + str(start_neighbors_idx))
        # print("start_neighbors_cost: " + str(start_neighbors_cost))
        # print("goal_neighbors_idx: " + str(goal_neighbors_idx))
        # print("goal_neighbors_cost: " + str(goal_neighbors_cost))

        result_traj = [] ### the output we want to construct
        isPathValid = False
        print("current planning query: ", self.query_idx)
        violated_edges = [] ### initially there are no violated edges

        counter = 0
        while (isPathValid == False):
            counter += 1
            ### trigger new call within the same query idx
            # start_time = time.time()
            searchSuccess, path =  self.serviceCall_astarPathFinding(
                    violated_edges, initialConfig, targetConfig, 
                    start_neighbors_idx, goal_neighbors_idx,
                    start_neighbors_cost, goal_neighbors_cost,
                    robot, workspace, armType)
            # print("Time for service call for astarPathFinding: {}".format(time.time() - start_time))
            if searchSuccess == False:
                print("the plan fails at the " + str(counter) + "th trial...")
                ### the plan fails, could not find a solution
                self.query_idx += 1
                return result_traj ### an empty trajectory
            ### otherwise, we need collision check and smoothing (len(path) >= 3)
            # start_time = time.time()
            smoothed_path, isPathValid, violated_edges = self.smoothPath(
                    path, initialConfig, targetConfig, object_idx, robot, workspace, armType)
            # print("Time for smooth the path: {}".format(time.time() - start_time))            

        ### congrats, the path is valid and finally smoothed, let's generate trajectory
        print("smoothed path: ", smoothed_path)
        ### directly generate trajectory based on the new path
        for i in range(0, len(smoothed_path)-1):
            if i == 0:
                config1 = initialConfig
            else:
                config1 = self.nodes[armType][smoothed_path[i]]
            if i == (len(smoothed_path)-2):
                config2 = targetConfig
            else:
                config2 = self.nodes[armType][smoothed_path[i+1]]
            ### get edge trajectory
            config_edge_traj = self.generateTrajectory_DirectConfigPath(config1, config2, robot, armType, object_idx, workspace)
            # result_traj.append(config_edge_traj)
            result_traj += config_edge_traj

        ### before you claim the victory of this query, increment the planning query
        ### so as to tell people this query is over, next time is a new query
        self.query_idx += 1

        return result_traj


    def smoothPath(self, path, initialConfig, targetConfig, object_idx, robot, workspace, armType):
        ### This function tries to smooth the given path
        ### output: a smooth path [a list of indexes] and whether the path is valid or not
        smoothed_path = []
        violated_edges = []
        start_idx = 0
        startNode_idx = path[start_idx] ### start
        smoothed_path.append(startNode_idx)
        curr_idx = start_idx + 1
        while (curr_idx < len(path)):
            currNode_idx = path[curr_idx]
            ### check edge validity between start_idx and curr_idx
            if start_idx == 0:
                config1 = initialConfig
            else:
                config1 = self.nodes[armType][startNode_idx]
            if curr_idx == (len(path)-1):
                config2 = targetConfig
            else:
                config2 = self.nodes[armType][currNode_idx]

            if (start_idx == 0 and curr_idx == 1) or (start_idx == len(path)-2 and curr_idx == len(path)-1):
                ### no need to check the edge validity between neighboring nodes
                ### which has either start or goal
                isEdgeValid = True
            else:
                ### check the edge
                isEdgeValid, FLAG = self.checkEdgeValidity_DirectConfigPath(
                                config1, config2, object_idx, robot, workspace, armType)
            if isEdgeValid:
                validFromStart_idx = curr_idx
                validNodeFromStart_idx = currNode_idx
                ### move on to the next node
                curr_idx += 1
                continue
            else:
                if (curr_idx - start_idx == 1):
                    print("Edge invalid, we need call A* again with the change of edge information")
                    print("FLAG: " + str(FLAG))
                    print(str(startNode_idx) + "," + str(currNode_idx))
                    edge = Edge()
                    edge.idx1 = startNode_idx
                    edge.idx2 = currNode_idx
                    violated_edges.append(edge)
                    smoothed_path = [] ### turn it back to empty path
                    return smoothed_path, False, violated_edges
                ### the edge is not valid
                ### add validNodeFromStart_idx to the smoothed_path
                smoothed_path.append(validNodeFromStart_idx)
                ### set validNodeFromStart_idx as the new start
                ### and the curr_idx does not change
                start_idx = validFromStart_idx
                startNode_idx = validNodeFromStart_idx
        smoothed_path.append(validNodeFromStart_idx)

        return smoothed_path, True, violated_edges


    def connectToNeighbors(self, config, object_idx, robot, workspace, armType):
        ### This function makes connections 
        ### between the specified config to neighboring nodes in the roadmap
        ### It returns (1) whether the config can be connected to any neighbors (isConfigValid)
        ###            (2) the neighbors it connects with
        neighbors_idx = []
        neighbors_cost = []
        connectSuccess = False

        dist = [utils.calculateNorm2(config, neighborConfig) for neighborConfig in self.nodes[armType]]
        neighborIndex, neighborDist = zip(*sorted(enumerate(dist), key=itemgetter(1)))
        neighborIndex = list(neighborIndex)
        neighborDist = list(neighborDist)

        # max_neighbors = self.num_neighbors
        max_neighbors = 5
        max_candiates_to_consider = self.num_neighbors

        ####### now connect potential neighbors for the specified config #######
        neighbors_connected = 0
        for j in range(max_candiates_to_consider):
            ### first check if the query node has already connected to enough neighbors
            if neighbors_connected >= max_neighbors:
                break
            ### otherwise, find the neighbor
            neighbor = self.nodes[armType][neighborIndex[j]]
            ### check the edge validity
            isEdgeValid, FLAG = self.checkEdgeValidity_DirectConfigPath(
                    config, neighbor, object_idx, robot, workspace, armType)
            # print("FLAG: " + str(FLAG))
            if isEdgeValid:
                neighbors_idx.append(neighborIndex[j])
                neighbors_cost.append(neighborDist[j])
                neighbors_connected += 1
        
        ### check if the number of neighboring connections is zero
        print("Number of neighbors for current node: " + str(neighbors_connected))
        if neighbors_connected != 0: connectSuccess = True
        return connectSuccess, neighbors_idx, neighbors_cost


    def findNeighborsForStartAndGoal(self,
                    initialConfig, targetConfig, object_idx, robot, workspace, armType): 
        ### return four things
        ### (1) start_neighbors_idx (a list of integer)
        ### (2) goal_neighbors_idx (a list of integer)
        ### (3) start_neighbors_cost (a list of float)
        ### (4) goal_neighbors_cost (a list of float)
        start_neighbors_idx = []
        goal_neighbors_idx = []
        start_neighbors_cost = []
        goal_neighbors_cost = []

        dist_to_start = [
            utils.calculateNorm2(initialConfig, neighborConfig) for neighborConfig in self.nodes[armType]]

        neighborIndex_to_start, neighborDist_to_start = zip(
                                    *sorted(enumerate(dist_to_start), key=itemgetter(1)))        
        neighborIndex_to_start = list(neighborIndex_to_start)
        neighborDist_to_start = list(neighborDist_to_start)
        dist_to_goal = [
            utils.calculateNorm2(targetConfig, neighborConfig) for neighborConfig in self.nodes[armType]]
        neighborIndex_to_goal, neighborDist_to_goal = zip(
                                    *sorted(enumerate(dist_to_goal), key=itemgetter(1)))
        neighborIndex_to_goal = list(neighborIndex_to_goal) 
        neighborDist_to_goal = list(neighborDist_to_goal)        

        # max_neighbors = self.num_neighbors
        max_neighbors = 5
        max_candiates_to_consider = self.num_neighbors

        ####### now connect potential neighbors for the start and the goal #######
        ### for start
        # print("for start")
        neighbors_connected_start = 0
        for j in range(max_candiates_to_consider):
            ### first check if the query node has already connected to enough neighbors
            if neighbors_connected_start >= max_neighbors:
                break
            ### otherwise, find the neighbor
            neighbor = self.nodes[armType][neighborIndex_to_start[j]]
            ### check the edge validity
            isEdgeValid, FLAG = self.checkEdgeValidity_DirectConfigPath(
                initialConfig, neighbor, object_idx, robot, workspace, armType)
            # print("FLAG: " + str(FLAG))
            if isEdgeValid:
                start_neighbors_idx.append(neighborIndex_to_start[j])
                start_neighbors_cost.append(neighborDist_to_start[j])
                neighbors_connected_start += 1
        print("Number of neighbors for start node: " + str(neighbors_connected_start))

        ### for goal
        # print("for goal")
        neighbors_connected_goal = 0
        for j in range(max_candiates_to_consider):
            ### first check if the query node has already connected to enough neighbors
            if neighbors_connected_goal >= max_neighbors:
                break
            ### otherwise, find the neighbor
            neighbor = self.nodes[armType][neighborIndex_to_goal[j]]
            ### check the edge validity
            isEdgeValid, FLAG = self.checkEdgeValidity_DirectConfigPath(
                targetConfig, neighbor, object_idx, robot, workspace, armType)
            # print("FLAG: " + str(FLAG))
            if isEdgeValid:
                goal_neighbors_idx.append(neighborIndex_to_goal[j])
                goal_neighbors_cost.append(neighborDist_to_goal[j])
                neighbors_connected_goal += 1
        print("Number of neighbors for goal node: " + str(neighbors_connected_goal))

        return start_neighbors_idx, goal_neighbors_idx, start_neighbors_cost, goal_neighbors_cost


    def attachObject(self, object_idx, workspace, robot, armType):
        if armType == "Left" or armType == "Left_torso":
            self.isObjectInLeftHand = True
            self.objectInLeftHand = workspace.object_geometries[object_idx].geo ### mesh 
        if armType == "Right" or armType == "Right_torso":
            self.isObjectInRightHand = True
            self.objectInRightHand = workspace.object_geometries[object_idx].geo ### mesh
        ### get the object pose relative to the frame of the end effector 
        ### since once the object is attached to the end effector, 
        ### it will remain constant (may add some noice)
        if armType == "Left" or armType == "Left_torso":
            curr_ee_pose = robot.left_ee_pose
            ls = p.getBasePositionAndOrientation(
                    self.objectInLeftHand, physicsClientId=self.planningServer)
            curr_object_global_pose = [list(ls[0]), list(ls[1])]
        if armType == "Right" or armType == "Right_torso":
            curr_ee_pose = robot.right_ee_pose
            ls = p.getBasePositionAndOrientation(
                    self.objectInRightHand, physicsClientId=self.planningServer)
            curr_object_global_pose = [list(ls[0]), list(ls[1])]
        
        inverse_ee_global = p.invertTransform(curr_ee_pose[0], curr_ee_pose[1])
        temp_localPose = p.multiplyTransforms(
            list(inverse_ee_global[0]), list(inverse_ee_global[1]),
            curr_object_global_pose[0], curr_object_global_pose[1])
        temp_localPose = [list(temp_localPose[0]), list(temp_localPose[1])]

        if armType == "Left" or armType == "Left_torso":
            self.leftLocalPose = temp_localPose
        if armType == "Right" or armType == "Right_torso":
            self.rightLocalPose = temp_localPose


    def detachObject(self, workspace, robot, armType):
        if armType == "Left" or armType == "Left_torso":
            self.isObjectInLeftHand = False
            self.objectInLeftHand = None
        if armType == "Right" or armType == "Right_torso":
            self.isObjectInRightHand = False
            self.objectInRightHand = None

    
    def attachObject_loadmap(self, workspace, robot, armType):
        if armType == "Right" or armType == "Right_torso":
            self.isObjectInRightHand = True
            ### specify the pre-computed local pose
            self.rightLocalPose = [[0.09468472003936768, 0.0007766783237457275, -0.0014880895614624023], \
                        [0.0924038216471672, -0.700919508934021, -0.09246741980314255, 0.7011584639549255]]
            ####### use that local pose to generate the sample_object #######
            ### get the global pose of the object
            sample_object_global_pose = self.getObjectGlobalPose(self.rightLocalPose, robot.right_ee_pose)
            sample_object_c = p.createCollisionShape(shapeType=p.GEOM_CYLINDER, radius=workspace.cylinder_radius, 
                        height=workspace.cylinder_height, physicsClientId=self.planningServer)
            sample_object_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                radius=workspace.cylinder_radius, length=workspace.cylinder_height, 
                rgbaColor=[1.0, 0.0, 0.0, 0.8], physicsClientId=self.planningServer)
            sample_objectM = p.createMultiBody(
                baseCollisionShapeIndex=sample_object_c, baseVisualShapeIndex=sample_object_v,
                basePosition=sample_object_global_pose[0], baseOrientation=sample_object_global_pose[1], 
                physicsClientId=self.planningServer)
            self.objectInRightHand = sample_objectM
    
    def detachObject_loadmap(self, armType):
        if armType == "Right" or armType == "Right_torso":
            self.isObjectInRightHand = False
            p.removeBody(self.objectInRightHand)
            self.objectInRightHand = None

    # def samplesConnect_advanced(self, robot, workspace, armType):
    #     connectivity = np.zeros((self.nsamples, self.nsamples))
    #     tree = spatial.KDTree(self.nodes[armType]) ### use KD tree to arrange neighbors assignment
    #     connectionsFile = self.roadmapFolder + "/connections_" + str(armType) + ".txt"
    #     f_connection = open(connectionsFile, "w")
    #     ### for each node
    #     for node_idx in range(len(self.nodes[armType])):
    #         queryNode = self.nodes[armType][node_idx]
    #         knn = tree.query(queryNode, k=self.num_neighbors, p=2)

    #         neighbors_connected = 0
    #         ### for each potential neighbor
    #         for j in range(len(knn[1])):
    #             ### first check if this query node has already connected to enough neighbors
    #             if neighbors_connected >= self.num_neighbors:
    #                 break
    #             if knn[1][j] == node_idx:
    #                 ### if the neighbor is the query node itself
    #                 continue
    #             if connectivity[node_idx][knn[1][j]] == 1:
    #                 ### the connectivity has been checked before
    #                 neighbors_connected += 1
    #                 continue
    #             ### Otherwise, check the edge validity
    #             ### in terms of collision with the robot itself and all known geometries (e.g. table/shelf)
    #             ### between the query node and the current neighbor
    #             neighbor = self.nodes[armType][knn[1][j]]
    #             isEdgeValid, FLAG = self.checkEdgeValidity_knownGEO(queryNode, neighbor, robot, workspace, armType)
    #             if not isEdgeValid:
    #                 ### we don't allow any collision with known geometry when there is no object in hand
    #                 continue
    #             ### you are reaching here as the edge pass the collision check 
    #             ### with robot self-collision and robot-known_geometry
    #             ### now check the collision between the robot and position candidate
    #             isEdgeValid, FLAG = self.checkEdgeValidity_ positionCandidates_labeledRoadmap(
    #                 queryNode, neighbor, robot, workspace, armType)

