#!/usr/bin/env python


class DFSSolver(object):

    def __init__(self, n_objects, valid_states, state_values):
        self.n_objects = n_objects
        self.all_objects = [i+1 for i in range(n_objects)]
        self.valid_states = valid_states
        self.state_values = state_values
        self.object_ordering = []
        self.object_values = []
        print("all_objects: " + str(self.all_objects) + "\n")

    def DFS(self):
        print("object_ordering: " + str(self.object_ordering))
        print("object_values: " + str(self.object_values))
        FLAG = False
        if (len(self.object_ordering) == self.n_objects): 
            return True
        for obj_idx in self.all_objects:
            if obj_idx in self.object_ordering:
                continue
            else:
                ### the object has not been consider before
                ### let's consider it ~~
                result_ordering = self.object_ordering + [obj_idx]
                if (self.valid_states[tuple(result_ordering)] == True):
                    self.object_ordering.append(obj_idx)
                    self.object_values.append(self.state_values[tuple(result_ordering)])
                    ### recursive call ~~
                    FLAG = self.DFS()
                    if FLAG: return FLAG
        ### if there is no option, before returning back
        ### pop the last element on the object_ordering
        if self.object_ordering != []:
            self.object_ordering.pop(-1)
            self.object_values.pop(-1)
        
        return FLAG

if __name__ == '__main__':
    
    n_objects = 3

    valid_states = {}
    valid_states[()] = True
    valid_states[(1,)] = True
    valid_states[(2,)] = True
    valid_states[(3,)] = True
    valid_states[(1,2)] = True
    valid_states[(2,1)] = True
    valid_states[(1,3)] = False
    valid_states[(3,1)] = True
    valid_states[(2,3)] = False
    valid_states[(3,2)] = True
    valid_states[(1,2,3)] = False
    valid_states[(1,3,2)] = False
    valid_states[(2,1,3)] = False
    valid_states[(2,3,1)] = False
    valid_states[(3,1,2)] = True
    valid_states[(3,2,1)] = True

    state_values = {}
    state_values[()] = 0,0
    state_values[(1,)] = 1.5
    state_values[(2,)] = 1.6
    state_values[(3,)] = 1.7
    state_values[(1,2)] = 2.3
    state_values[(2,1)] = 2.4
    state_values[(1,3)] = 2.5
    state_values[(3,1)] = 2.6
    state_values[(2,3)] = 2.7
    state_values[(3,2)] = 2.8
    state_values[(1,2,3)] = 6.0
    state_values[(1,3,2)] = 6.2
    state_values[(2,1,3)] = 6.4
    state_values[(2,3,1)] = 6.6
    state_values[(3,1,2)] = 6.8
    state_values[(3,2,1)] = 7.0

    dfs_solver = DFSSolver(n_objects, valid_states, state_values)
    search_success = dfs_solver.DFS()
    print('----------------------------')
    print("search success: " + str(search_success))
    print("object_ordering: " + str(dfs_solver.object_ordering))
    print("object_values: " + str(dfs_solver.object_values))
    




