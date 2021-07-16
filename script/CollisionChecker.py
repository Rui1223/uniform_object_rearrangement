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
        robotGEO, object_geometries, object_idx, isObjectInLeftHand, isObjectInRightHand):
        ### here object_geometries is a dictionary (key: object_index, value: objectGEO)
        # print("+++++++++++++++++++")
        # print("isObjectInLeftHand: ", isObjectInLeftHand)
        # print("isObjectInRightHand: ", isObjectInRightHand)
        isCollision = False
        ### loop through all object geometries in the workspace
        for obj_idx, object_geo in object_geometries.items():
            contacts = p.getClosestPoints(bodyA=robotGEO, bodyB=object_geo, distance=0.0, physicsClientId=self.server)
            # contacts = p.getContactPoints(robotGEO, object_geo, physicsClientId=self.server)
            if len(contacts) != 0:
                for contact in contacts:
                    if contact[8] >= 0:
                        ### This is a fake collision (>=0: separation, <0: penetration)
                        continue
                    ### for all objects
                    if contact[3] == 10 or contact[3] == 20:
                        ### we allow the object to be slightly contact with the end effector
                        # print("we allow the object to be slightly contact with the end effector")
                        continue
                    ### for the specified object
                    if (obj_idx == object_idx):
                        if isObjectInLeftHand == True:
                            ### you are checking the object which is in the left hand
                            ### in this case, ignore the collision between the left end effector and the object
                            if (contact[3] == 9 or contact[3] == 10): continue
                        if isObjectInRightHand == True:
                            ### you are checking the object which is in the right hand
                            ### in this case, ignore the collision between the right end effector and the object
                            if (contact[3] >= 19): continue

                    ### reach here since none of the tolerance case above meets
                    isCollision = True
                    # print("******robot collides with object " + str(obj_idx) + "******")
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

    
    def collisionCheck_object_objectGEO(self, objectGEO, object_geometries):
        isCollision = False
        ### loop through all object in objectGEOs
        for obj_idx, object_geo in object_geometries.items():
            contacts = p.getClosestPoints(
                bodyA=objectGEO, bodyB=object_geo, distance=0.003, physicsClientId=self.server)
            if len(contacts) != 0:
                isCollision = True
                # for contact in contacts:
                #     print("******moving object collides with object " + str(obj_idx) + "******")
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