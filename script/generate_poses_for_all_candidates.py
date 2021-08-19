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

### This file generates multiple poses for
### each position candidate

def main(args):
    print("Let's generate multiple poses for each position candidate")
    pybullet_plan_scene = PybulletPlanScene(args)

    ### first geneate all position candidates
    # pybullet_plan_scene.workspace_p.generateCandidatesGeometry()
    pybullet_plan_scene.generatePosesForAllCandidates("Right_torso")
    # pybullet_plan_scene.generatePoses_IKdateSet("Right_torso")

    time.sleep(10000)



if __name__ == '__main__':
    main(sys.argv)