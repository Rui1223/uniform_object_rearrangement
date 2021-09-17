# Robot Prehensile Rearrangement in Cluttered and Confined Spaces

### Project Description
This research project falls in the domain of uniform-shaped object rearrangement in cluttered and confined workspaces such as shelves, where overhand grasps are not possible. 
As a result, robot-object and object-object interactions occur frequently and have to be avoided so as to successfully complete a rearrangement task.
Therefore, it is a harder setup than the widely-researched tabletop ones, where robot-object and object-object interactions can be simplified or even waived.

Below is a simulated example of a Motoman SDA10F robot rearranging cylindrical objects in a cluttered and confined space (a cubic workspace with transparent glasses). The robot can only access objects from one side of the workspace and the task is to rearrange all the objects so that the objects with the same color are aligned in the same column (similar to a grocery scenario where commercial products of the same category are rearranged to be aligned after customers randomly drop them somewhere). The robotic physics simulator is [Pybullet](https://pybullet.org/wordpress/)
<img src="image_materials/rearrangement_example.gif" />

### Getting Started
The software infrastructure is developed in Ubuntu 20.04 with ROS Noetic and Python 3.8.10.
The robot physics simulator is Pybullet 3.1.3. Pybullet can be downloaded [here].(https://pypi.org/project/pybullet/)
Below are the addtional dependencies you need to install so as to run the code smoothly.
matplotlib <br/>
numpy>=1.11.0 <br/>
scipy <br/>
IPython <br/>
pickle <br/>
It could be difficult to exhaustively list all potential dependencies. Should you have any difficulties trying the software, do not hesitate to contact wrui1223@gmail.com for further help.

### Instruction
Once you download the repository, you need to create a ROS workspace where this repository fits in as a ROS package. Instructions on how to create a ROS workspace can be found [here]. (http://wiki.ros.org/catkin/Tutorials/create_a_workspace). Once you have created a workspace, say you name the workspace as 'catkin_ws', then go to the directory of the workspace and build code in the catkin workspace with
'catkin_make'
It may throw out some minor errors and if this is the case, repeat the 'catkin_make' two or three more times should work. (At least it works in my case. Again, feel free to contact wrui1223@gmail.com for further help.)
Once the 'catkin_make' is successful, to try an example on any existing method, run the following
'roslaunch uniform_object_rearrangement run_example.launch run_example:="<#object> <instance id> <generate or load an instance> <time allowed> <method name>"'
