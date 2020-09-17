# DWM1001 Project
This project is designed to interface with the [DWM1001](https://www.decawave.com/product/mdek1001-deployment-kit/) RFID tag to serve as an enemy locator.

## Dependencies

The code in this project has the following dependecies:

1. [CMake v2.8.12+](https://cmake.org/download/)
2. [dlib library v19.19+](http://dlib.net/ )
3. [ROS Melodic](https://www.ros.org/ )

External repositories:

1. [davemers0160 common code repository](https://github.com/davemers0160/Common )
2. [davemers0160 dlib-contrib repository](https://github.com/davemers0160/dlib-contrib )

The dwm_wrapper is a catkin package, and depends on the following ROS packages:

   - catkin
   - roscpp
   - rosconsole
   - std_msgs
   - geometry_msgs
   - message_runtime
   - message_generation


## ROS

### Run the program

To launch the dwm_wrapper from `~/catkin_ws` use:

  $ roslaunch src/robot/dwm1001/launch/dwm.launch


### Parameters

To configure the parameters of the dwm_wrapper node you can modify the configuration YAML file in the `params` folder.  The parameters that can be changed are:

  - serial_port: the name of the serial port to connect to
  - max_observations:  the maximum number of observations to use to calculate the enemy locations
  - min_range: the minimum allowed distance between observations to be used in the location calculation

## Non-ROS

In addition to this project being a ROS catkin package the code can also be compiled under Windows and Linux w/o ROS.

## Building the project

The project uses CMake as the primary mechanism to build the executables.  There are some modifications that may have to be made to the CMakeLists.txt file in order to get the project to build successfully.

The first thing that must be done is to create an environment variable called "PLATFORM".  The CMakeLists.txt file uses this variable to determine where to look for the other required repositories and/or libraries.  These will be machine specific.

To create an environment variable in Windows (drop the -m if you do not have elevated privileges):
```
setx -m PLATFORM MY_PC
```

In Linux (usually placed in .profile or .bashrc):
```
export PLATFORM=MY_PC
```

In the CMakeLists.txt file make sure to add a check for the platform you've added and point to the right locations for the repositories/libraries.

### Windows

From the directory that contains this file, execute the following commands in a Windows command window:

```
mkdir build
cd build
cmake -G "Visual Studio 15 2017 Win64" -T host=x64 ..
cmake --build . --config Release
```

Or you can use the cmake-gui and set the "source code" location to the location of the CmakeLists.txt file and the set the "build" location to the build folder. Then you can open the project in Visual Studio and compile from there.

### Linux

From the directory that contains this file, execute the following commands in a terminal window:

```
mkdir build
cd build
cmake ..
cmake --build . --config Release -- -j4
```

Or you can use the cmake-gui and set the "source code" location to the location of the CmakeLists.txt file and the set the "build" location to the build folder. Then open a terminal window and navigate to the build folder and execute the follokwing command:

```
cmake --build . --config Release -- -j4
```

The -- -j4 tells the make to use 4 cores to build the code.  This number can be set to as many cores as you have on your PC.

## Running

The code is run by supplying all of the parameters in a single file.  Using this method all of the input parametes must be supplied and they must be in the order outlined in the example file *sample_input_file.txt*

To use the file enter the following:

```
Windows: dwm_ex
Linux: ./dwm_ex
```

