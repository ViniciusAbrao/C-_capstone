# C-_capstone
Udacity C++ Capstone Project
C++ Capstone Project

Writeup – Vinicius Abrão da Silva Marques

In this Capstone Project I implemented a C++ ROS Node simulated in Gazebo/RVIZ. The main objective is to subscribe the /scan topic, that contains the laser measurement in front of the robot, and to print the value in a progress bar. The progress bar presents the measured value of the laser scan and its variation in a scale from 0 to 30 meters, the maximum value read by the laser.

In order to test the C++ ROS node, I have used the robot that I developed in the Nanodegree Robotics Software Engineering.

This Writeup is divided in three parts: The first describes the ROS project; The second describes the C++ code implemented in this capstone project; The last part describes how to launch the files.

1- ROS Project:

The following setup has been used to configure the workspace:
sudo apt-get install xterm
pip install rospkg
sudo apt-get install ros-kinetic-slam-gmapping

PS.: THIS IS A ROS BASED PROJECT, THEREFORE IT IS NECESSARY TO INSTALL ROS AND ITS DEPENDENCIES. IN THE VIRTUAL MACHINE OF WORKSPACE, I HAVE LAUNCHED IT WITH ROS KINETIC, ALREADDY INSTALLED.

The packages used for the project are:
- gmapping: used for mapping
- amcl: localization
- move_base: used for the path planning (navigation)
- teleop: used to move the robot as a joystick
- my_robot: this is my project package that includes launch files, configs, shell scripts, environment and robot model
- add_markers: package used to create the markers in the Rviz
- pick objects: package used to move the robot from a pickup position to a drop off position

My launch files are found at the folder:
.../catkin_ws/src/my_robot/launch
I have recorded a video showing how to run the code. You can check it at:
https://youtu.be/ESEcfpf5kfo

2- C++ code (It is found at:
/home/workspace/catkin_ws/src/my_robot/src/my_subs.cpp):

The following rubric items are addressed in the code:
-Compiling and Testing: In order to compile it, I have made the necessary changes in the “... /my_robot/CmakeLists.txt” file.

-README: This pdf file.

- The project demonstrates an understanding of C++ functions and control structures. For Loop: Line 101-105. Also I divided the code in two classes and the main function that receives the inputs (int argc, char **argv) – Line 112.

- The project uses Object Oriented Programming techniques: Two classes are implemented in lines 5 (class MyMovableClass) and 85 (class PrintData).

- Classes use appropriate access specifiers for class members: All class data members are explicitly specified as public, protected, or private. Lines 7, 11, 87 and 93.

- The project follows the Rule of 5: Lines 15, 22, 28, 36, 48, 57.

- The project uses smart pointers instead of raw pointers: Line 149.

- The project uses multithreading: It is used the callBack function of  ros::Subscriber method together with boost::bind, that enable to pass the shared pointer and its function (counterCallback) to store the laser data as soon as a new data is available. Line 151.

3- Launch the project:

- In terminal 1:
roslaunch my_robot world.launch
- In terminal 2:
roslaunch my_robot amcl.launch
- In terminal 3:
rosrun my_robot my_subs
- After that, you may give the navigation goal command at RVIZ to move the robot to a desired position. According to the motion of the robot, you may check in the terminal 3 the sensor laser measurement printed with the progress bar.

Ps-1.: - In the virtual machine I have found sometimes an error when open gazebo. I believe that is because it is missing a GPU or some graphics configuration. In that case of fail in opening gazebo, I closed and reopened the Desktop GUI and retyped the commands above, and it works correctly.

Ps-2.: Since this is a ROS project It is necessary when open each terminal to type:
cd /home/workspace/catkin_ws
source devel/setup.bash

Ps-3.: I am not mention the step by step to install and configure ROS, since this is not the focus of this project. More information can be found at:
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
