#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

import time
import sys
import os

from PybulletPlanScene import PybulletPlanScene

import rospy
import rospkg

### This file generates a labeled roadmap for 
### left/right arm (include torso) of the motoman robot

def main(args):
    print("Let's generate the labeled roadmap for motoman")
    pybullet_plan_scene = PybulletPlanScene(args)

    nsamples = int(args[1])

    ### generate samples
    pybullet_plan_scene.planner_p.generateSamples(
        nsamples, pybullet_plan_scene.robot_p, pybullet_plan_scene.workspace_p)
    
    ### Now generate object geometries on all position candidates
    pybullet_plan_scene.workspace_p.generateCandidatesGeometry_labeledRoadmap()
    pybullet_plan_scene.planner_p.samplesConnect_advanced(
        pybullet_plan_scene.robot_p, pybullet_plan_scene.workspace_p, "Right_torso")


if __name__ == '__main__':
    main(sys.argv)