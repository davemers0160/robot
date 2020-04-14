#!/bin/bash
# Install Robot Operating System (ROS) on NVIDIA Jetson AGX Xavier

## ----------------------------------------------------------------------------
# add the main repos 
sudo apt-add-repository universe
sudo apt-add-repository multiverse
sudo apt-add-repository restricted


sudo apt-get update -y
sudo apt-get upgrade -y

sudo apt-get install -y lsb-release wget less udev apt-transport-https nano cmake cmake-gui usbutils git build-essential && \
     libusb-dev curl putty software-properties-common iputils-ping
     
#sudo apt-get install -y libqt5xml5 libxmu-dev libxi-dev libturbojpeg


## ----------------------------------------------------------------------------
## Get the ZED SDK - ZED_SDK_Tegra_JP43_v3.1.2.run
wget https://stereolabs.sfo2.digitaloceanspaces.com/zedsdk/3.1/ZED_SDK_Tegra_JP43_v3.1.2.run
chmod +x ZED_SDK_Tegra_JP43_v3.1.2.run
sudo ./ZED_SDK_Tegra_JP43_v3.1.2.run silent
#rm -rf /var/lib/apt/lists/*
sudo mkdir -p /root/Documents/ZED/
sudo usermod -a -G video $USER

## ----------------------------------------------------------------------------
## Setup and install ROS
current_deb=$DEBIAN_FRONTEND
sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime
export DEBIAN_FRONTEND=noninteractive
sudo apt-get install -y tzdata
dpkg-reconfigure --frontend noninteractive tzdata

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update -y

sudo apt-get install -y ros-melodic-desktop-full 
sudo apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool

sudo rosdep init
sudo rosdep update
echo "source /opt/$USER/melodic/setup.bash" >> /home/$USER/.bashrc

export DEBIAN_FRONTEND=$current_deb


## ----------------------------------------------------------------------------
## Install some python packages
curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py"
python get-pip.py

pip install launchpadlib pandas Cython contextlib2 pillow lxml jupyter matplotlib utils --user
pip install tensorflow-gpu==1.14 tensorflow_hub --user --ignore-installed



## ----------------------------------------------------------------------------
## clone the ROS wrapper packages
mkdir -p /home/$USER/catkin_ws
mkdir -p /home/$USER/catkin_ws/src
git clone -b 'v3.0.3' --single-branch https://github.com/stereolabs/zed-ros-wrapper.git /home/$USER/catkin_ws/src/zed-ros-wrapper
git clone https://github.com/davemers0160/robot.git /home/$USER/catkin_ws/src/robot


## ----------------------------------------------------------------------------
## get the required support repos
cd /home/$USER
mkdir -p /home/$USER/Projects
cd Projects
git clone https://github.com/davemers0160/Common.git
    
    

# /bin/bash -c "source /opt/ros/melodic/setup.bash; cd /home/$USER/catkin_ws; catkin_make"
source /opt/ros/melodic/setup.bash
cd /home/$USER/catkin_ws
catkin_make


## ----------------------------------------------------------------------------
## get a couple of object detection models
cd /home/$USER
git clone https://github.com/tensorflow/models /home/$USER/models

cd /home/$USER/models/research
wget -O protobuf.zip https://github.com/google/protobuf/releases/download/v3.0.0/protoc-3.0.0-linux-x86_64.zip
unzip protobuf.zip ./bin/protoc object_detection/protos/*.proto --python_out=.

cd /home/$USER
wget http://download.tensorflow.org/models/object_detection/ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03.tar.gz
tar -xzvf ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03.tar.gz
mv ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03 ssd_resnet50_v1_fpn
rm ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03.tar.gz

wget http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v2_oid_v4_2018_12_12.tar.gz
tar -xzvf ssd_mobilenet_v2_oid_v4_2018_12_12.tar.gz
mv ssd_mobilenet_v2_oid_v4_2018_12_12 ssd_mobilenet_v2_oid
rm ssd_mobilenet_v2_oid_v4_2018_12_12.tar.gz
    
wget http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12.tar.gz
tar -xzvf faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12.tar.gz
mv faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12 faster_rcnn_inception_resnet_v2_oid
rm faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12.tar.gz




#mkdir tf_obj_det
#cd tf_obj_det
#wget https://raw.githubusercontent.com/davemers0160/robot/master/python_code/ros_tf_obj_det.py


    

## ----------------------------------------------------------------------------
## put the everything in the right paths
echo "export PYTHONPATH=\$PYTHONPATH:/home/\$USER/models:/home/\$USER/models/research:/home/\$USER/models/research/slim:/home/\$USER/models/research/object_detection" >> /home/$USER/.bashrc
echo "PATH=\$PATH:/home/\$USER/.local/bin" >> /home/$USER/.bashrc
echo "source /home/\$USER/catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc
echo "export PLATFORM=JAX" >> /home/$USER/.bashrc


echo "Installation complete!"



