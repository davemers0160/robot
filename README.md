# Director's Cup - Xavier Branch

This code repository is the official code for the 2020 Director's Cup Robotic Challenge.  The code is targeted towards the Nvidia Jetson Xavier running the Nvidia Jetpack 4.3.

The Jetson Xavier Jetpack needs to be installed first before the rest of the code can be installed.

The training of the DNN for the object detection was done using the following repo: [davemers0160 dlib_object_detection repository](https://github.com/davemers0160/dlib_object_detection ).  The network was trained using the RGB trainer.

## Dependencies

The code in this repository has the following dependecies:

1. [CMake v2.8.12+](https://cmake.org/download/ )
2. [dlib library v19.19+](http://dlib.net/ )
3. [ROS Melodic](https://www.ros.org/ )
4. [Nvidia Jetpack v4.3](https://developer.nvidia.com/embedded/jetpack )
5. [Stereolabs ZED SDK v3.0.1+](https://www.stereolabs.com/developers/)

External repositories:

1. [davemers0160 common code repository](https://github.com/davemers0160/Common )
2. [davemers0160 dlib-contrib repository](https://github.com/davemers0160/dlib-contrib )
3. [Stereolabs zed_ros_wrapper](https://github.com/stereolabs/zed-ros-wrapper )

## Repository Breakdown

### common

This folder contains the common code that is used between various projects.  This folder also contains the trained network weights files for the object detection challenge.

### dockerfiles

This folder contains the dockerfiles for simulating an Xavier.  This is used to install ROS and other software compenents without worrying about bricking the computer because ROS jacked something up.

### dwm1001

This folder contains the code for the DWM1001 RFID tag that is used as an enemy beacon locator.

### object_detect

This folder contains the code that performs object detection to identify boxes and backpacks using the ZED/ZED2 Stereolabs camera and the zed_ros_wapper.

### python_code

This folder contains some example python code for wire detection and object detection.

### zed_object_detect

This folder contains the code for that performs object detection to identify boxes and backpacks using the ZED/ZED2 Stereolabs camera and does not require the zed_ros_wrapper

### jax_install.sh

This files contains the configuration script to install the required libraries, download the correct repos and compile the code.

## Building the project

Open a terminal, clone the repository, update the dependencies and build the packages:

    $ mkdir catkin_ws
    $ mkdir catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/davemers0160/robot.git
    $ cd ../
    $ catkin_make -DCMAKE_BUILD_TYPE=Release -j6
    $ source ./devel/setup.bash

