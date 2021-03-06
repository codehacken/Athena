# ATHENA
The project is to create a Joint Model of Language and Perception for a Robotic Arm using Active Learning. We use an Amazon Echo and A Kinect Sensor in this project.

INSTALLATION
============
Pre-Requisities
===============
Install ROS and a compatible version of gazebo. For the purpose of this project, the current version of ROS used is Indigo and Gazebo 2.2.6 has been installed.

Installation
================
Create a catkin workspace by executing the following command:
```
$ cd src/
$ catkin_init_workspace
$ cd ..
$ catkin_make
```
This creates the workspace where the project can be executed.

EXECUTION
=========
The project is designed to work on both real sensors and in a simulated environment using Gazebo. To run it on Gazebo, execute the following commands. (Before executing, add the the current working directory to your environment by running the command in the main project folder ```$ source devel/setup.sh```

```
$ roslaunch jaco_kinect_gazebo kinect_world.launch
```
This will start Gazebo with the kinect_world environment.

NOTES
=====
## Integrate GAZEBO, ROS & HUSKY

1. [ROS-Husky Interface](http://www.clearpathrobotics.com/2014/03/ros-101-drive-husky/)

2. [Simulate Husky in Gazebo](http://www.clearpathrobotics.com/2013/11/husky-simulation-in-gazebo/)

3. [ROS-Gazebo Interface](http://www.generationrobots.com/en/content/75-gazebo-and-ros)

## Kinect2 (using libfreenect2)
1. https://github.com/code-iai/iai_kinect2

2. https://github.com/OpenKinect/libfreenect2

## OpenCV (use package vision_opencv)
1. http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html

2. http://wiki.ros.org/vision_opencv

3. http://codeyarns.com/2013/07/23/how-to-compile-opencv-without-cuda-support/

Compile openCV with the following command:
```
cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
```

Important Note: Use opencv 2.4 instead of 3.0 as cv_bridge on iai_kinect2 is incompatible with it (for now).

REFERENCE
==========
```
1. Matuszek, Cynthia, et al. "A joint model of language and perception for grounded attribute learning." arXiv preprint arXiv:1206.6423 (2012).
```

