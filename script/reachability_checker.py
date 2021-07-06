#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import time
import sys
import os

from MotomanRobot import MotomanRobot
from WorkspaceTable import WorkspaceTable
from Planner import Planner
from PybulletPlanScene import PybulletPlanScene

import rospy
import rospkg

### This file check the reachablity of the robot given
### the environment setting (table height + table distance)
### generate a discrete set of goal locations
### for each of the goal locations, multiple goals are test

def main(args):
    print("Let's explore the reachability")
    pybullet_plan_scene = PybulletPlanScene(args)

    ######## regular resolution reachability checking ###########
    pybullet_plan_scene.deployAllGoalPositions()
    orientations = pybullet_plan_scene.generateOrientations() ### generate all possible orientations
    ### pick an orientation
    orientation = orientations[1]
    pybullet_plan_scene.calculateReachabilityMap(orientation)
    #############################################################
    
    # ######## high resolution reachability checking ###########
    # pybullet_plan_scene.deployAllGoalPositions(
    #     object_interval=0.02, side_clearance=0.02, cylinder_radius=0.005, cylinder_height=0.25)
    # orientations = pybullet_plan_scene.generateOrientations() ### generate all possible orientations
    # ### pick an orientation
    # orientation = orientations[2]
    # pybullet_plan_scene.calculateReachabilityMap(orientation, "sphere")
    # ##########################################################

    time.sleep(10000)

if __name__ == '__main__':
    main(sys.argv)