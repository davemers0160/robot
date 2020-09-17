# Object Detect Project

This project is designed to interface with the Stereolabs ZED/ZED2 stereo camera using the zed_ros_wrapper package

## Dependencies

The code in this project has the following dependecies:

1. [CMake v2.8.12+](https://cmake.org/download/)
2. [dlib library v19.19+](http://dlib.net/ )
3. [ROS Melodic](https://www.ros.org/ )

External repositories:

1. [davemers0160 common code repository](https://github.com/davemers0160/Common )
2. [davemers0160 dlib-contrib repository](https://github.com/davemers0160/dlib-contrib )
3. [Stereolabs zed_ros_wrapper](https://github.com/stereolabs/zed-ros-wrapper )

The object_detect project is a catkin package, and depends on the following ROS packages:

   - catkin
   - roscpp
   - rosconsole
   - std_msgs
   - sensor_msgs
   - cv_bridge
   - message_filters
   - message_runtime
   - message_generation

## ROS

### Run the program

To launch the object_detect node from `~/catkin_ws` use:

    $ roslaunch src/robot/object_detect/launch/object_detect.launch

### Parameters

To configure the parameters of the dwm_wrapper node you can modify the configuration YAML file in the `params` folder.  The parameters that can be changed are:

  - loop_rate: The update rate for processing an image
  - img_topic: The topic to get the input RGB image
  - depth_topic: The topic to get the depth information
  - cam_info_topic: The topic to get the information on the camera
  - net_file: The absolute path to the network weights file
  - crop_x: The left pixel location for the image processing window
  - crop_y: The top pixel location for the image processing window
  - crop_w: The width of the crop window
  - crop_h: The height of the crop window


