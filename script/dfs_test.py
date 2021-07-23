#!/usr/bin/env python

all_objects = [1,2,3]
n_objects = len(all_objects)
object_ordering = []

def DFS(object_ordering):
    print("object_ordering: " + str(object_ordering))
    for obj_idx in all_objects:
        if obj_idx in object_ordering:
            continue
        else:
            ### the object has not been consider before
            object_ordering.append(obj_idx)
            DFS(object_ordering)
    ### if there is no option, before returning back
    ### pop the last element on the object_ordering
    if object_ordering != []:
        object_ordering.pop(-1)

DFS(object_ordering)
print("good luck")