# Group-8
IMT Nord Europe
# Challenge 1
The goal of the challenge is to demonstrate the capability of a robot to move in a cluttered environment with a possibility to see what the robot see.
The Group-8 directory allows the Kobuki robot to move around a room. Thanks to the different files, you will be able to execute an autonomous movement of the robot, scan a room, establish a map of a room and recognize various objects in its environment. You will also be able to visualize what the robot observes thanks to Rviz2, and to observe robot simulations thanks to Gazebo.

How to install the directory:
- Install ros2 and Python 3
- Create a workspace (directory)
- Open shell
- Return to your workspace
- clone and install tbot https://bitbucket.org/imt-mobisyst/mb6-tbot
- insert command git clone https://github.com/jpinheirolp/larm-mother.git


Attention:
Keep in mind that, before launching each ROS2 command, you must return to your ros2 directory and launch the colcon build command in your shell, then source install/setup.bash

Challange 1:
The goal of the challange is to ensure, that the robot can move around a area and avoiding the obstacles in there:
- sim.launch.py  
- tbot.launch.py 
- visualize.launch.py 

How to run the challenge 1 in the simulation:
- open shell
- copy and paste the folloing command: ros2 launch challenge_1 sim.launch.py

How to run the challenge 1 with a real robot:
- open the terminal
- copy and paste the folloing command: ros2 launch challenge_1 tbot.launch.py

How to run the visualization for challange 1:
- open the terminal
- copy and paste the folloing command: ros2 launch challenge_1 visualize.py




