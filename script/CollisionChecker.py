from __future__ import division
import pybullet as p
import pybullet_data
import IPython

class CollisionChecker(object):
    def __init__(self, server):
        self.server = server

    def collisionCheck_instance_cylinder_objects(self, object_geo, geometries, cylinder_radius):
        isCollision = False
        ### loop through all object geometries in the workspace
        for g in geometries:
            contacts = p.getClosestPoints(bodyA=object_geo, bodyB=g, distance=2*cylinder_radius, physicsClientId=self.server)
            # contacts = p.getContactPoints(object_geo, g, physicsClientId=self.server)
            if len(contacts) != 0:
                isCollision = True
                break
        
        return isCollision

    def collisionCheck_selfCollision(self, robotGEO):
        isCollision = False
        # contacts = p.getClosestPoints(bodyA=robotGEO, bodyB=robotGEO, distance=0.0, physicsClientId=self.server)
        contacts = p.getContactPoints(bodyA=robotGEO, bodyB=robotGEO, physicsClientId=self.server)
        if len(contacts) == 0:
            pass
        else:
            ### check each contact
            for contact in contacts:
                if (contact[3] == 23 and contact[4] == 25) or (contact[3] == 28 and contact[4] == 30) \
                    or (contact[3] == 24 and contact[4] == 25) or (contact[3] == 29 and contact[4] == 30) \
                    or (contact[3] == 6 and contact[4] == 9) or (contact[3] == 19 and contact[4] == 16) \
                    or (contact[3] == 22 and contact[4] == 25):
                    ### we can allow collisions among the links of the robotiq hand 
                    ### (mounted on the right arm)
                    ### we also allow collision between the gripper and the wrist (both left and right hand)
                    pass
                else:
                    isCollision = True
                    # print("******robot self collision!******")
                    # print("body-to-body collision: ")
                    # print(str(contact[1]) + ": " + str(contact[2]))
                    # print("link-to-link collision: ")
                    # print(str(contact[3]) + ": " + str(contact[4]))
                    # print("contact position on robotGEO")
                    # print(str(contact[5]))
                    # print("contact position on robotGEO")
                    # print(str(contact[6]))
                    # print("contact distance:")
                    # print(str(contact[8]))
                    break

        return isCollision

    def collisionCheck_robot_knownGEO(self, robotGEO, knownGEO):
        isCollision = False
        ### loop through all known geometries in the workspace
        for g in knownGEO:
            contacts = p.getClosestPoints(bodyA=robotGEO, bodyB=g, distance=0.0, physicsClientId=self.server)
            # contacts = p.getContactPoints(robotGEO, g, physicsClientId=self.server)
            if len(contacts) != 0:
                isCollision = True
                # print("******robot collision with known GEO******")
                # for contact in contacts:
                #     print("body-to-body collision: ")
                #     print(str(contact[1]) + ": " + str(contact[2]))
                #     print("link-to-link collision: ")
                #     print(str(contact[3]) + ": " + str(contact[4]))
                #     print("contact position on robotGEO")
                #     print(str(contact[5]))
                #     print("contact position on knownGEO")
                #     print(str(contact[6]))
                #     print("contact distance:")
                #     print(str(contact[8]))
                break

        return isCollision


    def collisionCheck_robot_objectGEO(self, 
                robotGEO, objectGEO, armType, isObjectInLeftHand, isObjectInRightHand):
        # print("+++++++++++++++++++")
        # print("isObjectInLeftHand: ", isObjectInLeftHand)
        # print("isObjectInRightHand: ", isObjectInRightHand)
        isCollision = False
        ### loop through all object geometries in the workspace
        for g in objectGEO:
            contacts = p.getClosestPoints(bodyA=robotGEO, bodyB=g, distance=0.0, physicsClientId=self.server)
            # contacts = p.getContactPoints(robotGEO, g, physicsClientId=self.server)
            if len(contacts) != 0:
                for contact in contacts:
                    if contact[8] >= 0:
                        ### This is a fake collision (>=0: separation, <0: penetration)
                        continue
                    if (contact[3] == 10 and (armType == "Left" or armType == "Left_torso")) or \
                            (contact[3] == 20 and (armType == "Right" or armType == "Right_torso")):
                        ### we allow the object to be slightly contact with the end effector
                        # print("we allow the object to be slightly contact with the end effector")
                        pass
                    elif (contact[3] == 9 and (armType == "Left" or armType == "Left_torso") and contact[8] >= 0) or \
                            (contact[3] == 19 and (armType == "Right" or armType == "Right_torso") and contact[8] >= 0):
                        ### we allow the object to be slightly contact with the hand
                        # print("we allow the object to be slightly contact with the hand")
                        pass
                    else:
                        isCollision = True
                        # print("******robot collides with object GEO******")
                        # print("body-to-body collision: ")
                        # print(str(contact[1]) + ": " + str(contact[2]))
                        # print("link-to-link collision: ")
                        # print(str(contact[3]) + ": " + str(contact[4]))
                        # print("contact position on robotGEO")
                        # print(str(contact[5]))
                        # print("contact position on objectGEO")
                        # print(str(contact[6]))
                        # print("contact distance:")
                        # print(str(contact[8]))
                        break
        return isCollision


    def collisionCheck_object_knownGEO(self, objectGEO, knownGEO):
        isCollision = False
        ### loop through all objectGEO and knownGEO
        for known_g in knownGEO:
            contacts = p.getClosestPoints(
                bodyA=objectGEO, bodyB=known_g, distance=-0.003, physicsClientId=self.server)
            # contacts = p.getContactPoints(object_g, known_g, physicsClientId=self.server)
            if len(contacts) != 0:
                isCollision = True
                # for contact in contacts:
                #     print("******moving object collides with known GEO******")
                #     print("body-to-body collision: ")
                #     print(str(contact[1]) + ": " + str(contact[2]))
                #     print("link-to-link collision: ")
                #     print(str(contact[3]) + ": " + str(contact[4]))
                #     print("contact position on robotGEO")
                #     print(str(contact[5]))
                #     print("contact position on objectGEO")
                #     print(str(contact[6]))
                #     print("contact distance:")
                #     print(str(contact[8]))
                break

        return isCollision

    
    def collisionCheck_object_objectGEO(self, objectGEO, objectGEOs):
        isCollision = False
        ### loop through all object in objectGEOs
        for object_g in objectGEOs:
            contacts = p.getClosestPoints(
                bodyA=objectGEO, bodyB=object_g, distance=0.003, physicsClientId=self.server)
            if len(contacts) != 0:
                isCollision = True
                # for contact in contacts:
                #     print("******moving object collides with object GEO******")
                #     print("body-to-body collision: ")
                #     print(str(contact[1]) + ": " + str(contact[2]))
                #     print("link-to-link collision: ")
                #     print(str(contact[3]) + ": " + str(contact[4]))
                #     print("contact position on robotGEO")
                #     print(str(contact[5]))
                #     print("contact position on objectGEO")
                #     print(str(contact[6]))
                #     print("contact distance:")
                #     print(str(contact[8]))
                break

        return isCollision