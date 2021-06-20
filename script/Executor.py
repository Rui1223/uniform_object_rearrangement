from __future__ import division
import pybullet as p
import pybullet_data

import time
import math

import utils
from CollisionChecker import CollisionChecker

import rospy
from sensor_msgs.msg import JointState

class Executor(object):
    def __init__(self, server,
        isObjectInLeftHand=False, isObjectInRightHand=False,
        objectInLeftHand=None, objectInRightHand=None):
        self.server = server
        self.isObjectInLeftHand = isObjectInLeftHand
        self.isObjectInRightHand = isObjectInRightHand
        self.objectInLeftHand = objectInLeftHand
        self.objectInRightHand = objectInRightHand
        self.leftLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]
        self.rightLocalPose = [[-1, -1, -1], [-1, -1, -1, -1]]

    def getObjectGlobalPose(self, local_pose, ee_global_pose):
        """ get object global pose given 
        (1) local pose and (2) global pose of end effector
        """
        temp_object_global_pose = p.multiplyTransforms(
            ee_global_pose[0], ee_global_pose[1],
            local_pose[0], local_pose[1])
        object_global_pose = [list(temp_object_global_pose[0]), list(temp_object_global_pose[1])]

        return object_global_pose        

    def updateRealObjectBasedonLocalPose(self, robot, armType):
        """update the object based on local pose 
        between the gripper and the object
        inputs
        ======
            robot (robot_e): the robot
            armType (string): "Left"/"Right"/"Left_torso"/"Right_torso"
        outputs
        =======
            None
        """
        if armType == "Left" or armType == "Left_torso":
            object_global_pose = self.getObjectGlobalPose(self.leftLocalPose, robot.left_ee_pose)
            p.resetBasePositionAndOrientation(
                self.objectInLeftHand, object_global_pose[0], object_global_pose[1], 
                physicsClientId=self.planningServer)
        if armType == "Right" or armType == "Right_torso":
            object_global_pose = self.getObjectGlobalPose(self.rightLocalPose, robot.right_ee_pose)
            p.resetBasePositionAndOrientation(
                self.objectInRightHand, object_global_pose[0], object_global_pose[1], 
                physicsClientId=self.planningServer)

    def executeTrajectory(self, traj, robot, armType):
        """the function to execute the trajectory with robot arm
        in the real / execution scene
        inputs
        ======
            traj (JointState[]): a list of JointState
            robot (robot_e): the robot on which the trajectory is executed
            armType (string): specified arm (e.g., "Left" or "Right_torso")
        outputs
        =======
            None
        """
        for joint_state in traj:
            if armType == "Left" or armType == "Right":
                robot.moveSingleArm(joint_state.position, armType)
            if armType == "Left_torso" or armType == "Right_torso":
                robot.moveSingleArm_torso(joint_state.position[1:8], joint_state.position[0], armType)
            ### if the object is in hand, it's a in-hand manipulation
            ### also move the manipulated object as well
            if (self.isObjectInLeftHand and (armType == "left" or armType == "Left_torso") or \
                self.isObjectInRightHand and (armType == "Right" or armType == "Right_torso")):
                self.updateRealObjectBasedonLocalPose(robot, armType)
            time.sleep(0.1) ### you can tune the time here to make it more real-time control

    