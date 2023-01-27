# Group-8
IMT Nord Europe 
Leonardo Ivens, Erik Kohlmann
# Challenge 1
The goal of the challenge is to demonstrate the capability of a robot to move in a cluttered environment with a possibility to see what the robot see.
The Group-8 directory allows the Kobuki robot to move around a room. Thanks to the different files, you will be able to execute an autonomous movement of the robot, scan a room, establish a map of a room and recognize various objects in its environment. You will also be able to visualize what the robot observes thanks to Rviz2, and to observe robot simulations thanks to Gazebo.


## How to install the directory:
- Install ros2 and Python 3
- Create a workspace (directory)
- Open shell
- Return to your workspace
- clone and install tbot https://bitbucket.org/imt-mobisyst/mb6-tbot
- insert command git clone https://github.com/jpinheirolp/larm-mother.git


### Attention:
Keep in mind that, before launching each ROS2 command, you must return to your ros2 directory and launch the colcon build command in your shell, then source install/setup.bash


## Challange 1:
The goal of the challange is to ensure, that the robot can move around a area and avoiding the obstacles in there:
- command to launch callenge 1 in a simulation: `sim.launch.py`  
- command to launch the real robot: `tbot.launch.py` 
- command to to start the rviz2 with the setting for challenge 1: `visualize.launch.py` 

### How to run the challenge 1 in the simulation:
- open terminal
- copy and paste the folloing command: `ros2 launch challenge_1 sim.launch.py`

### How to run the challenge 1 with a real robot:
- open the terminal
- copy and paste the folloing command: `ros2 launch challenge_1 tbot.launch.py`

### How to run the visualization for challange 1:
- open the terminal
- copy and paste the folloing command: `ros2 launch challenge_1 visualize.py`

## Challenge 2"
The goal of this challenge is to make the robot able to creat a map with all obstacles in a specific area. The robot is also able to detect an orange bottle. Each time the robot detect a orange bottle, it displays a detection message.

### How to run the challenge 2 with a real robot:
- open the terminal
- copy and paste the folloing command: `ros2 launch challenge_2 tbot.launch.py`
- open another terminal and copy and paste the following command: `ros2 launch challenge_2 rviz2.launch.py`

## What is missing?
So far the robot is able to detect an orange bottle, but there are some problems to detect a black bottle. This part of the code has to be improved. After detecting a bottle the robot should move close to the detected bottle to make sure, that this is a real bottle. Another problem is to open rviz2 on the remote PC. 

## Nest steps:
After the robot is able to detect the two different types of bottles, the next step is to localize the bottles. There for the robot has to detect, localize and then display the location of the bottle in a map on rviz2. 


Here is the link for the video we made to explain the project and its demonstration: `https://drive.google.com/drive/folders/1--yTILK23CtbdRagHEa3vF0LbCXpAEjk` 



