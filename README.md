# Robot Prehensile Rearrangement in Cluttered and Confined Spaces

### Project Description
This research project falls in the domain of uniform-shaped object rearrangement in cluttered and confined workspaces such as shelves, where overhand grasps are not possible. 
As a result, robot-object and object-object interactions occur frequently and have to be avoided so as to successfully complete a rearrangement task.
Therefore, it is a harder setup than the widely-researched tabletop ones, where robot-object and object-object interactions can be simplified or even waived.

Below is a simulated example of a Motoman SDA10F robot rearranging cylindrical objects in a cluttered and confined space (a cubic workspace with transparent glasses). The robot can only access objects from one side of the workspace and the task is to rearrange all the objects so that the objects with the same color are aligned in the same column (similar to a grocery scenario where commercial products of the same category are rearranged to be aligned after customers randomly drop them somewhere). The robotic physics simulator is [Pybullet](https://pybullet.org/wordpress/).

<img src="image_materials/rearrangement_example.gif" />

**Real robot demo videos:**

[![IMAGE ALT TEXT](http://img.youtube.com/vi/iBIkDog2caU/0.jpg)](https://www.youtube.com/watch?v=iBIkDog2caU&ab_channel=PRACSYS)
[![IMAGE ALT TEXT](http://img.youtube.com/vi/EUeo8PUi3HQ/0.jpg)](https://www.youtube.com/watch?v=EUeo8PUi3HQ&ab_channel=PRACSYS)


### Getting Started
The software infrastructure is developed in Ubuntu 20.04 with ROS Noetic and Python 3.8.10.
The robot physics simulator is Pybullet 3.1.3. Pybullet can be downloaded [here](https://pypi.org/project/pybullet/).
Below are the addtional dependencies you need to install so as to run the code smoothly.
matplotlib <br/>
Numpy>=1.11.0 <br/>
OpenCV 4.2.0 <br/>
Scipy <br/>
IPython <br/>
Pickle <br/>
It could be difficult to exhaustively list all potential dependencies. Should you have any difficulties trying the software, do not hesitate to contact wrui1223@gmail.com for further help.

### Instruction
Once you download the repository, you need to create a ROS workspace where this repository fits in as a ROS package. Instructions on how to create a ROS workspace can be found [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
After you create a workspace, say you name the workspace as `catkin_ws`, go to the directory of the workspace and build code in the catkin workspace by running <br/>
`catkin_make` <br/>
It may throw out some minor errors and if this is the case, repeat the `catkin_make` two or three more times should work. (At least it works in my case. Again, feel free to contact wrui1223@gmail.com for further help.) <br/>
Once the `catkin_make` is successful, to try an example on any existing method, run the following <br/>
`roslaunch uniform_object_rearrangement run_example.launch run_example:="<#object> <instance_id> <generate/load an instance> <time_allowed> <method_name>"` <br/>
Here the placedholders in <> are </br>
- **<#object>**: the number of object you want to try (options 6-12)
- **<instance_id>**: which instance do you want to try (an integer)
- **<generate/load an instance>**: 'g': indicates generating a new instance; 'l': indicates loading an existing instance
- **<time_allowed>**: the time allowed for the method to solve the instance/problem (time suggestion: 180 or 360 seconds)
- **<method_name>**: indicates the name of the method you want to try (options: CIRS, CIRS_nonlabeled, DFSDP, DFSDP_nonlabeled, mRS, mRS_nonlabeled). CIRS is the best method and is recommended.

In summary, if you run <br/>
`roslaunch uniform_object_rearrangement run_example.launch run_example:="6 1 l 180 CIRS"` <br/>
you are loading an existing example in the instance folder named "examples/6/1" (1st instance in the 6-object scenario) and use CIRS as the method to solve it given a limitation of 180 seconds.

if you run <br/>
`roslaunch uniform_object_rearrangement run_example.launch run_example:="8 3 g 180 CIRS"` <br/>
you are generating a new example as the 3rd instance in the 8-object scenario and use CIRS as the method to solve it given a limitation of 180 seconds.

It will launch two interfaces (1) for an execution scene, which describes the real scene where the objects and the robot reside, and (2) for a planning scene, which describes how the robot thinks of the enviroment and use it for planning. You will see the robot working in the planning scene to search for a solution given the instance.

- If a solution is found, the message on the terminal first asks you if you want to save the instance (y/n). If you type 'y', the instance is saved in the corresponding instance folder with the name "instance_info.txt" containing the example information. This message only pops out if you are generating a new instance.
- Then it asks if you want to execute the solution (y/n). If you type 'y', the solution (a sequence of manipulation paths) will be executed in the execution scene. 
- After the execution, it will ask if you want to save the solution path (y/n). If you type 'y', a file named "path.obj" will be saved in the same instance folder.
- At the end, it will ask if you want to save the object ordering for future reference. If you type 'y', a flie named "ordering.txt" will be saved in the same instance folder containing the ordering with which objects are rearranged so as to solve the problem.

You can also run the following <br/>
`roslaunch uniform_object_rearrangement execute_task_plan.launch execute_task_plan:="<#object> <instance_id>"` <br/>
to just execute a solution path you have saved before. 

For instance, if you run <br/>
`roslaunch uniform_object_rearrangement execute_task_plan.launch execute_task_plan:="6 1"` <br/>
then the solution path in "examples/6/1/path.obj" will be executed in the execution scene.

Should you have any questions, feel free to contact wrui1223@gmail.com for references.
