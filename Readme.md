# Robotics, Vision and Control - Assignment 10

Final project of "Robotics, Vision and Control" course. The goal is to program a pick and place task of a UR5 robot mounted on a mobile platform using a RealSense camera. 

In our task the platform must be fixed and only the UR5 can move to pick the 4 cubes that are above the wall. To find the correct position and orientation of each cube we use aruco marker. Of course, to give the correct target to the robot these information are transformed with respect to the base frame of our robot. In order to "see" each aruco, we defined a point in 3d where the robot can see it and once detected it moves towards it and then go back in an home position to repeat again the procedure for next cube.

The trajectories implemented are both in operational(linear) space and configurational space. We use the firt to go from the detection point to the cube. Instead, the second is used to raise up the robot at the beginning of the task and to reach the home configuration. 

The work is based on ROS. This package, which includes trajectory and camera control, has been developed in C++.

## Clarifications

To use this package you need the kairosim workspace which includes the model of robot and the camera. 

After that, you can create a new workspace that control kairosim.

The main file is talker.cpp from which you can launch all the task. The other classes manage the trajectory(operational and confgiurational), the kinematics(based on KDL) and the aruco which are detected by the camera. 

Remeber to launch the kairosim before this package, otherwise talker.cpp cannot find the topics to comunicate. 
