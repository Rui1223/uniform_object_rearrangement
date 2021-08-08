#!/usr/bin/env python
from __future__ import division

import pybullet as p
import pybullet_data

from collections import OrderedDict
import os
import math
import numpy as np
import time
import IPython
from random import uniform, random

from CollisionChecker import CollisionChecker
import utils

from uniform_object_rearrangement.msg import CylinderObj

### This file defines workspace for the table ###

class WorkspaceTable(object):
    def __init__(self, 
        rosPackagePath, robotBasePosition,
        standingBase_dim, table_dim, table_offset_x, 
        mesh_path, isPhysicsTurnOn, server):
        ### get the server
        self.server = server
        self.rosPackagePath = rosPackagePath
        self.mesh_path = mesh_path
        self.known_geometries = []
        self.object_geometries = OrderedDict()
        self.createRobotStandingBase(robotBasePosition, standingBase_dim, isPhysicsTurnOn)
        self.createTableScene(robotBasePosition, table_dim, table_offset_x, isPhysicsTurnOn)
        self.table_offset_x = table_offset_x
        self.collisionAgent = CollisionChecker(self.server)

        ### RED, GREEN, BLUE, YELLOW, MAGENTA,
        ### ORANGE, BROWN, BLACK, GREY,
        ### CYAN, PURPLE, LIME,
        ### MAROON, OLIVE, TEAL, NAVY,
        ### PINK, APRICOT, BEIGE,
        ### MINT, LAVENDER
        ### (21 colors up to 21 objects)

        self.color_pools = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1],
                        [0.96, 0.51, 0.19], [0.604, 0.388, 0.141], [0, 0, 0], [0.66, 0.66, 0.66],
                        [0.259, 0.831, 0.957], [0.567, 0.118, 0.706], [0.749, 0.937, 0.271],
                        [0.502, 0, 0], [0.502, 0.502, 0], [0.275, 0.6, 0.565], [0, 0, 0.459],
                        [0.98, 0.745, 0.831], [1.0, 0.847, 0.694], [1.0, 0.98, 0.784],
                        [0.667, 1.0, 0.765], [0.863, 0.745, 1.0]]

    def createRobotStandingBase(self, robotBasePosition, standingBase_dim, isPhysicsTurnOn):
        ################ create the known geometries - standingBase  ####################
        self.standingBase_dim = np.array(standingBase_dim)
        self.standingBasePosition = [
            robotBasePosition[0], robotBasePosition[1], robotBasePosition[2]-self.standingBase_dim[2]/2-0.005]
        self.standingBase_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.server)
        self.standingBase_v = p.createVisualShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.standingBase_dim/2, physicsClientId=self.server)
        if isPhysicsTurnOn == True:
            self.standingBaseM = p.createMultiBody(
                baseCollisionShapeIndex=self.standingBase_c, baseVisualShapeIndex=self.standingBase_v,
                basePosition=self.standingBasePosition, physicsClientId=self.server)
        else:
            self.standingBaseM = p.createMultiBody(
                baseCollisionShapeIndex=self.standingBase_c, baseVisualShapeIndex=self.standingBase_v,
                basePosition=self.standingBasePosition, physicsClientId=self.server)            
        print("standing base: " + str(self.standingBaseM))
        self.known_geometries.append(self.standingBaseM)
        #################################################################################

    def createTableScene(self, 
        robotBasePosition, table_dim, table_offset_x, isPhysicsTurnOn):

        ################ create the known geometries - table  ###########################
        print("---------Enter to table scene!----------")
        # self.table_dim = np.array([table_dim[0], table_dim[1], table_dim[2]+self.standingBase_dim[2]+0.005])
        self.table_dim = np.array([table_dim[0], table_dim[1], table_dim[2]])
        self.tablePosition = [
            table_offset_x+self.table_dim[0]/2, robotBasePosition[1], robotBasePosition[2]+(self.table_dim[2]/2-self.standingBase_dim[2]-0.005)]

        self.table_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                halfExtents=self.table_dim/2, physicsClientId=self.server)
        self.table_v = p.createVisualShape(shapeType=p.GEOM_BOX,
                                halfExtents=self.table_dim/2, physicsClientId=self.server)
        self.tableM = p.createMultiBody(baseCollisionShapeIndex=self.table_c, baseVisualShapeIndex=self.table_v,
                                            basePosition=self.tablePosition, physicsClientId=self.server)
        print("table: " + str(self.tableM))
        self.known_geometries.append(self.tableM)

        print("inner edge of the table: ")
        print(self.tablePosition[0]-table_dim[0]/2)
        print("table surface: ")
        print(self.tablePosition[2]+table_dim[2]/2)
        print("left side of the table: ")
        print(self.tablePosition[1]+table_dim[1]/2)
        print("right side of the table: ")
        print(self.tablePosition[1]-table_dim[1]/2)
        #################################################################################

    def addConstrainedArea(self, ceiling_height, thickness_flank):
        self.ceiling_height = ceiling_height
        self.thickness_flank = thickness_flank
        print("---------Add constrained area!----------")
        ################ add a constrained area - a transparent box  ####################
        self.flankColor = [111/255.0, 78/255.0, 55/255.0, 0.3]
        self.sideFlank_dim = np.array([self.table_dim[0], thickness_flank, ceiling_height])
        self.backFlank_dim = np.array([thickness_flank, self.table_dim[1]-thickness_flank*2, ceiling_height])
        self.topFlank_dim = np.array([self.table_dim[0], self.table_dim[1], thickness_flank])
        self.sideFlank_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.sideFlank_dim/2, physicsClientId=self.server)
        self.sideFlank_v = p.createVisualShape(shapeType=p.GEOM_BOX,
                            halfExtents=self.sideFlank_dim/2, rgbaColor=self.flankColor, physicsClientId=self.server)
        self.leftFlankPosition = [self.tablePosition[0],
                                  self.tablePosition[1] + self.table_dim[1] / 2 - thickness_flank / 2,
                                  self.tablePosition[2] + self.table_dim[2] / 2 + self.sideFlank_dim[2] / 2]
        self.rightFlankPosition = [self.tablePosition[0],
                                  self.tablePosition[1] - self.table_dim[1] / 2 + thickness_flank / 2,
                                  self.tablePosition[2] + self.table_dim[2] / 2 + self.sideFlank_dim[2] / 2]
        self.leftFlankM = p.createMultiBody(
            baseCollisionShapeIndex=self.sideFlank_c, baseVisualShapeIndex=self.sideFlank_v,
            basePosition=self.leftFlankPosition, physicsClientId=self.server)
        self.rightFlankM = p.createMultiBody(
            baseCollisionShapeIndex=self.sideFlank_c, baseVisualShapeIndex=self.sideFlank_v,
            basePosition=self.rightFlankPosition, physicsClientId=self.server)
        self.known_geometries.append(self.leftFlankM)
        self.known_geometries.append(self.rightFlankM)

        self.backFlank_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.backFlank_dim/2, physicsClientId=self.server)
        self.backFlank_v = p.createVisualShape(shapeType=p.GEOM_BOX,
                            halfExtents=self.backFlank_dim/2, rgbaColor=self.flankColor, physicsClientId=self.server)
        self.backFlankPosition = [self.tablePosition[0] + self.table_dim[0] / 2 - thickness_flank / 2,
                                  self.tablePosition[1], self.tablePosition[2] + self.table_dim[2] / 2 + self.backFlank_dim[2] / 2]
        self.backFlankM = p.createMultiBody(
            baseCollisionShapeIndex=self.backFlank_c, baseVisualShapeIndex=self.backFlank_v,
            basePosition=self.backFlankPosition, physicsClientId=self.server)
        self.known_geometries.append(self.backFlankM)

        self.topFlank_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                    halfExtents=self.topFlank_dim/2, physicsClientId=self.server)
        self.topFlank_v = p.createVisualShape(shapeType=p.GEOM_BOX,
                            halfExtents=self.topFlank_dim/2, rgbaColor=self.flankColor, physicsClientId=self.server)
        self.topFlankPosition = [self.tablePosition[0], self.tablePosition[1], 
                                 self.tablePosition[2] + self.table_dim[2] / 2 + ceiling_height + thickness_flank / 2]
        self.topFlankM = p.createMultiBody(
            baseCollisionShapeIndex=self.topFlank_c, baseVisualShapeIndex=self.topFlank_v,
            basePosition=self.topFlankPosition, physicsClientId=self.server)
        self.known_geometries.append(self.topFlankM)

        self.constrained_area_center = [self.tablePosition[0]-thickness_flank/2, 0.0]
        self.constrained_area_x_limit = [self.tablePosition[0]-self.table_dim[0]/2, \
                                         self.tablePosition[0]+self.table_dim[0]/2-thickness_flank]
        self.constrained_area_y_limit = [self.tablePosition[1]-self.table_dim[1]/2+thickness_flank, \
                                         self.tablePosition[1]+self.table_dim[1]/2-thickness_flank]
        self.constrained_area_dim = [self.constrained_area_x_limit[1]-self.constrained_area_x_limit[0], \
                                     self.constrained_area_y_limit[1]-self.constrained_area_y_limit[0]]

        print("left flank: " + str(self.leftFlankM))
        print("right flank: " + str(self.rightFlankM))
        print("back flank: " + str(self.backFlankM))
        print("top flank: " + str(self.topFlankM))
        # print("constrained_area_center: ", str(self.constrained_area_center))
        # print("constrained_area_x_limit", str(self.constrained_area_x_limit))
        # print("constrained_area_y_limit", str(self.constrained_area_y_limit))
        # print("constrained_area_dim", str(self.constrained_area_dim))
        ###########################################################################################

    def setDeploymentParam(self, 
            cylinder_radius, cylinder_height, discretization_x, discretization_y, 
            object_interval_x, object_interval_y, side_clearance_x, side_clearance_y):
        self.cylinder_radius = cylinder_radius
        self.cylinder_height = cylinder_height
        self.discretization_x = discretization_x
        self.discretization_y = discretization_y
        self.object_interval_x = object_interval_x
        self.object_interval_y = object_interval_y
        self.side_clearance_x = side_clearance_x
        self.side_clearance_y = side_clearance_y

    def generateInstance_cylinders(self, num_objects):
        ### return: success (bool)

        self.num_objects = num_objects ### obtain the number of objects
        print("--------generate an instance---------")
        cylinder_c = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
                                    radius=self.cylinder_radius, height=self.cylinder_height, physicsClientId=self.server)

        for obj_i in range(self.num_objects):
            isCollision = True
            timeout = 20
            while isCollision and timeout > 0:
                print("generate the {}th object".format(obj_i))
                timeout -= 1
                ### generate the center of an object with uniform distribution
                ### uniform_x(0.89, 1.235), uniform_y(-0.605, 0.585)
                # start_pos = [
                #     round(uniform(self.constrained_area_x_limit[0] + 0.02 + self.cylinder_radius, \
                #                     self.constrained_area_x_limit[1] - self.side_clearance_x - self.cylinder_radius - 0.02), 3),
                #     round(uniform(self.constrained_area_y_limit[0] + self.side_clearance_y + self.cylinder_radius, \
                #                     self.constrained_area_y_limit[1] - self.side_clearance_y - self.cylinder_radius - 0.02), 3),
                #     round(self.tablePosition[2] + self.table_dim[2] / 2 + self.cylinder_height / 2, 3)
                # ]
                # rgbacolor = [round(random(), 3), round(random(), 3), round(random(), 3), 1.0]

                ### uniform_x(0.87, 1.255), uniform_y(-0.605, 0.605)
                start_pos = [
                    round(uniform(self.constrained_area_x_limit[0] + self.cylinder_radius, \
                                    self.constrained_area_x_limit[1] - self.side_clearance_x - self.cylinder_radius), 3),
                    round(uniform(self.constrained_area_y_limit[0] + self.side_clearance_y + self.cylinder_radius, \
                                    self.constrained_area_y_limit[1] - self.side_clearance_y - self.cylinder_radius), 3),
                    round(self.tablePosition[2] + self.table_dim[2] / 2 + self.cylinder_height / 2, 3)
                ]
                rgbacolor = self.color_pools[obj_i] + [1.0] ### No transparency
                cylinder_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                                radius=self.cylinder_radius, length=self.cylinder_height, rgbaColor=rgbacolor, physicsClientId=self.server)
                cylinder_objectM = p.createMultiBody(
                    baseCollisionShapeIndex=cylinder_c, baseVisualShapeIndex=cylinder_v,
                    basePosition=start_pos, physicsClientId=self.server)
                isCollision = self.collisionAgent.collisionCheck_instance_cylinder_objects(
                                        cylinder_objectM, [obj_geo.geo for obj_geo in self.object_geometries.values()], self.cylinder_radius)
                if isCollision:
                    ### remove that mesh as it is invalid (collision with existing object meshes)
                    p.removeBody(cylinder_objectM)
            
            if isCollision:
                return False
            else:
                ### congrats, the object's location is accepted
                ### assign the start position to the closest candidate position
                start_position_idx = self.assignToNearestCandiate(start_pos)
                self.object_geometries[obj_i] = CylinderObject(
                            obj_i, start_pos, start_position_idx, cylinder_objectM, self.cylinder_radius, self.cylinder_height, rgbacolor)

        ### assign goal positions
        self.assignGoalPositions()
        for obj_idx, position_idx in self.all_goal_positions.items():
            self.object_geometries[obj_idx].setGoalPosition(self.all_position_candidates[position_idx], position_idx) ### assignment goal position
            ### visualize it for confirmation
            temp_rgbacolor = self.color_pools[obj_idx] + [0.25]
            temp_cylinder_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                    radius=self.cylinder_radius, length=self.cylinder_height, rgbaColor=temp_rgbacolor, physicsClientId=self.server)
            temp_cylinder_objectM = p.createMultiBody(
                    baseVisualShapeIndex=temp_cylinder_v,
                basePosition=self.all_position_candidates[position_idx], physicsClientId=self.server)

        ############## printing test ##############
        for obj_idx in range(self.num_objects):
            print(obj_idx)
            print(self.object_geometries[obj_idx].curr_pos)
            print(self.object_geometries[obj_idx].curr_position_idx)
            print(self.object_geometries[obj_idx].goal_pos)
            print(self.object_geometries[obj_idx].goal_position_idx)
            print(self.object_geometries[obj_idx].rgbacolor)
        ###########################################
        
        return True


    def reproduceInstance_cylinders(self, cylinder_objects):
        ### input: cylinder_objects (CylinderObj[])
        ### output: success (bool)

        print("--------reproduce an instance---------")

        for cylinder_obj in cylinder_objects:
            cylinder_radius = round(cylinder_obj.radius, 3)
            cylinder_height = round(cylinder_obj.height, 3)
            cylinder_rgbacolor = [
                round(cylinder_obj.rgbacolor.r, 3), round(cylinder_obj.rgbacolor.g, 3), 
                round(cylinder_obj.rgbacolor.b, 3), round(cylinder_obj.rgbacolor.a, 3)]
            cylinder_curr_position = [cylinder_obj.curr_position.x, cylinder_obj.curr_position.y, cylinder_obj.curr_position.z]
            cylinder_curr_position_idx = cylinder_obj.curr_position_idx
            cylinder_goal_position = [cylinder_obj.goal_position.x, cylinder_obj.goal_position.y, cylinder_obj.goal_position.z]
            cylinder_goal_position_idx = cylinder_obj.goal_position_idx
            cylinder_c = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
                radius=cylinder_radius, height=cylinder_height, physicsClientId=self.server)
            cylinder_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                radius=cylinder_radius, length=cylinder_height, rgbaColor=cylinder_rgbacolor, physicsClientId=self.server)
            cylinder_objectM = p.createMultiBody(baseCollisionShapeIndex=cylinder_c, baseVisualShapeIndex=cylinder_v,
                basePosition=cylinder_curr_position, physicsClientId=self.server)
            self.object_geometries[cylinder_obj.obj_idx] = CylinderObject(
                cylinder_obj.obj_idx, cylinder_curr_position, cylinder_curr_position_idx, 
                cylinder_objectM, cylinder_obj.radius, cylinder_obj.height, cylinder_rgbacolor)
            self.object_geometries[cylinder_obj.obj_idx].setGoalPosition(cylinder_goal_position, cylinder_goal_position_idx)
        
        return True

    
    def obtainCylinderObjectsInfo(self):
        cylinder_objects = []
        for obj_i in range(self.num_objects):
            cylinder_object = CylinderObj()
            ### fill in the data
            cylinder_object.obj_idx = obj_i
            cylinder_object.curr_position.x = self.object_geometries[obj_i].curr_pos[0]
            cylinder_object.curr_position.y = self.object_geometries[obj_i].curr_pos[1]
            cylinder_object.curr_position.z = self.object_geometries[obj_i].curr_pos[2]
            cylinder_object.curr_position_idx = self.object_geometries[obj_i].curr_position_idx
            cylinder_object.goal_position.x = self.object_geometries[obj_i].goal_pos[0]
            cylinder_object.goal_position.y = self.object_geometries[obj_i].goal_pos[1]
            cylinder_object.goal_position.z = self.object_geometries[obj_i].goal_pos[2]
            cylinder_object.goal_position_idx = self.object_geometries[obj_i].goal_position_idx
            cylinder_object.radius = self.cylinder_radius
            cylinder_object.height = self.cylinder_height
            cylinder_object.rgbacolor.r = self.object_geometries[obj_i].rgbacolor[0]
            cylinder_object.rgbacolor.g = self.object_geometries[obj_i].rgbacolor[1]
            cylinder_object.rgbacolor.b = self.object_geometries[obj_i].rgbacolor[2]
            cylinder_object.rgbacolor.a = self.object_geometries[obj_i].rgbacolor[3]
            cylinder_objects.append(cylinder_object)
        return cylinder_objects


    def loadInstance_cylinders(self, num_objects, instance_number):

        instanceFile = os.path.join(self.rosPackagePath, "examples", str(num_objects)) + "/" + str(instance_number) + ".txt"
        print("--------load an instance---------")
        self.cylinder_c = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
                                    radius=self.cylinder_radius, height=self.cylinder_height, physicsClientId=self.server)
        self.num_objects = 0
        counter = 0
        f_instance = open(instanceFile, "r")
        for line in f_instance:
            line = line.split()
            if counter % 2 == 0:
                ### that's object index
                obj_idx = int(line[0])
            if counter % 2 == 1:
                ### that's current/start position for that object
                start_pos = [float(i) for i in line]
                ### create the visualization mesh of the object
                object_rgbacolor_start = self.color_pools[obj_idx] + [1.0]
                cylinder_start_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                    radius=self.cylinder_radius, length=self.cylinder_height, 
                    rgbaColor=object_rgbacolor_start, physicsClientId=self.server)
                cylinder_objectM_start = p.createMultiBody(
                    baseCollisionShapeIndex=self.cylinder_c, baseVisualShapeIndex=cylinder_start_v,
                    basePosition=start_pos, physicsClientId=self.server)
                start_position_idx = self.assignToNearestCandiate(start_pos)
                self.object_geometries[obj_idx] = CylinderObject(
                    obj_idx, start_pos, start_position_idx, cylinder_objectM_start, \
                        self.cylinder_radius, self.cylinder_height, object_rgbacolor_start)
                self.num_objects += 1
            ### before the next loop
            counter += 1
        f_instance.close()

        ### now assign the goal positions and visualize them
        self.assignGoalPositions()
        for obj_idx, position_idx in self.all_goal_positions.items():
            self.object_geometries[obj_idx].setGoalPosition(self.all_position_candidates[position_idx], position_idx)
            ### visualize it for confirmation
            object_rgbacolor_goal = self.color_pools[obj_idx] + [0.25]
            cylinder_goal_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                    radius=self.cylinder_radius, length=self.cylinder_height, 
                    rgbaColor=object_rgbacolor_goal, physicsClientId=self.server)
            cylinder_objectM_goal = p.createMultiBody(
                baseVisualShapeIndex=cylinder_goal_v,
                basePosition=self.all_position_candidates[position_idx], physicsClientId=self.server)

        ############## printing test ##############
        for obj_idx in range(self.num_objects):
            print(obj_idx)
            print(self.object_geometries[obj_idx].curr_pos)
            print(self.object_geometries[obj_idx].curr_position_idx)
            print(self.object_geometries[obj_idx].goal_pos)
            print(self.object_geometries[obj_idx].goal_position_idx)
            print(self.object_geometries[obj_idx].rgbacolor)
        ###########################################
        return True


    def deployAllPositionCandidates(self, discretization_x=None, discretization_y=None):
        ################ This function calculate all possible postions (x,y) ################
        ################### given the constrained area + discretization #####################
        if discretization_x == None: discretization_x = self.discretization_x
        if discretization_y == None: discretization_y = self.discretization_y
        self.all_position_candidates = OrderedDict()
        ### formula for compute n_y and n_x (number of objects in y and x direction)
        ### 0.02 + r + d_x*(n_x - 1) + (r + sc_x) = |x|
        ### (r + sc_y) + d_y*(n_y - 1) + (r + sc_y) = |y|
        self.n_candidates_x = \
            int(np.floor((self.constrained_area_dim[0] - 0.02 - self.side_clearance_x + discretization_x - 2*self.cylinder_radius) / discretization_x))
        self.n_candidates_y = \
            int(np.floor((self.constrained_area_dim[1] - 2*self.side_clearance_y + discretization_y - 2*self.cylinder_radius) / discretization_y))

        candidate_idx = 0
        for x_i in range(self.n_candidates_x):
            for y_j in range(self.n_candidates_y):
                candidate_pos = [
                    round(self.constrained_area_x_limit[0] + 0.02 + self.cylinder_radius + discretization_x * x_i, 3),
                    round(self.constrained_area_y_limit[0] + self.side_clearance_y + self.cylinder_radius + discretization_y * y_j, 3),
                    round(self.tablePosition[2] + self.table_dim[2] / 2 + self.cylinder_height / 2, 3)                    
                ]
                self.all_position_candidates[candidate_idx] = candidate_pos
                candidate_idx += 1
        self.positionCandidate_x_limit = [self.all_position_candidates[0][0], self.all_position_candidates[len(self.all_position_candidates)-1][0]]
        self.positionCandidate_y_limit = [self.all_position_candidates[0][1], self.all_position_candidates[len(self.all_position_candidates)-1][1]]
        # print("print all position candidates")
        # print(self.all_position_candidates)

    def assignGoalPositions(self, object_interval_x=None, object_interval_y=None):
        ### assign goal positions based on #objects, all_position_candidates
        ### as well as object_interval_x and object_interval_y
        if object_interval_x == None: object_interval_x = self.object_interval_x
        if object_interval_y == None: object_interval_y = self.object_interval_y
        self.all_goal_positions = OrderedDict()

        self.incremental_x = int(object_interval_x / self.discretization_x)
        self.incremental_y = int(object_interval_y / self.discretization_y)

        temp_row_start_idx = 0
        row_counter = 1
        candidate_idx = 0
        for obj_i in range(self.num_objects):
            self.all_goal_positions[obj_i] = candidate_idx
            ### move on to the next goal
            candidate_idx += self.incremental_y
            row_counter += self.incremental_y
            if row_counter > self.n_candidates_y:
                ### switch to the next row for goals
                candidate_idx = temp_row_start_idx + self.n_candidates_y * self.incremental_x
                temp_row_start_idx = candidate_idx
                row_counter = 1
        
        # print("all_goal_positions: ")
        # for obj_idx, position_idx in self.all_goal_positions.items():
        #     print(str(obj_idx) + ": " + str(position_idx))

    def assignToNearestCandiate(self, position):
        ### given a position (x,y,z), calculate the nearest candidate to that position
        ### and return the idx of the position candidate
        potential_neighbor_indexes = []
        x_value = math.floor((position[0]-self.positionCandidate_x_limit[0])/self.discretization_x)
        if x_value < 0:
            ### very close to constraint area x lower limit
            x_indexes = [0]
        elif x_value >= self.n_candidates_x - 1:
            ### very close to constraint area x upper limit
            x_indexes = [self.n_candidates_x - 1] 
        else:
            x_indexes = [x_value, x_value+1]
        y_value = math.floor((position[1]-self.positionCandidate_y_limit[0])/self.discretization_y)
        if y_value < 0:
            ### very close to constraint area y lower limit
            y_indexes = [0]
        elif y_value >= self.n_candidates_y - 1:
            ### very close to constraint area y upper limit
            y_indexes = [self.n_candidates_y - 1] 
        else:
            y_indexes = [y_value, y_value+1]

        for y_idx in y_indexes:
            for x_idx in x_indexes:
                potential_neighbor_indexes.append(x_idx*self.n_candidates_y+y_idx)
        ### find the nearest candidate
        nearest_candidate_dist = np.inf
        for neighbor_idx in potential_neighbor_indexes:
            temp_dist = utils.computePoseDist_pos(position, self.all_position_candidates[neighbor_idx])
            if temp_dist < nearest_candidate_dist:
                nearest_candidate_dist = temp_dist
                nearest_candidate_idx = neighbor_idx

        return nearest_candidate_idx


    def updateObjectMesh(self, obj_idx, position, position_idx, orientation=[0, 0, 0, 1]):
        ### input - target_pose: [x, y, z]
        p.resetBasePositionAndOrientation(
            self.object_geometries[obj_idx].geo, position, orientation, physicsClientId=self.server)
        self.object_geometries[obj_idx].curr_pos = position
        self.object_geometries[obj_idx].curr_position_idx = position_idx

    
    def generateCandidatesGeometry_labeledRoadmap(self):
        ''' This function generate object_geometries on all position candidates'''

        cylinder_c = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
            radius=self.cylinder_radius, height=self.cylinder_height, physicsClientId=self.server)
        rgbacolor = [0.5, 0.5, 0.5, 0.5]
        cylinder_v = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
            radius=self.cylinder_radius, length=self.cylinder_height, rgbaColor=[0.5, 0.5, 0.5, 0.5], physicsClientId=self.server)
        for candidate_idx, candidate_pos in self.all_position_candidates.items():
            cylinder_objectM = p.createMultiBody(
                baseCollisionShapeIndex=cylinder_c, baseVisualShapeIndex=cylinder_v, basePosition=candidate_pos, physicsClientId=self.server)
            self.object_geometries[candidate_idx] = CylinderObject(
                candidate_idx, candidate_pos, candidate_idx, cylinder_objectM, self.cylinder_radius, self.cylinder_height, rgbacolor)
        ### printing test
        # for candidate_idx, cylinder_object in self.object_geometries.items():
        #     print(str(candidate_idx) + ": " + str(cylinder_object.curr_pos))


### general class of object in the workspace
class AnyObject(object):
    def __init__(self, index, curr_pos, geo):
        self.object_index = index
        self.curr_pos = curr_pos
        self.geo = geo

    def setPos(pos):
        self.curr_pos = curr_pos
    
class CylinderObject(AnyObject):
    def __init__(self, index, curr_pos, curr_position_idx, geo, cylinder_radius, cylinder_height, rgbacolor):
        ### invoking the __init__ of the parent class
        AnyObject.__init__(self, index, curr_pos, geo)
        self.curr_position_idx = curr_position_idx
        self.cylinder_radius = cylinder_radius
        self.cylinder_height = cylinder_height
        self.rgbacolor = rgbacolor

    def setGoalPosition(self, goal_pos, goal_position_idx):
        self.goal_pos = goal_pos
        self.goal_position_idx = goal_position_idx