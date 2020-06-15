#!/bin/bash
# Install Robot Operating System (ROS) on NVIDIA Jetson AGX Xavier
# to run this script run the following from the home folder:
# wget https://raw.githubusercontent.com/davemers0160/robot/master/jax_install.sh
# chmod a+x jax_install.sh
# sudo ./jax_install.sh

# old school check for *nix echo styles
if [ "`echo -n`" = "-n" ]; then
  n=""
  c="\c"
else
  n="-n"
  c=""
fi

# menu for the interactive selection of which phases of the install occur
echo "-------------------------------------------------------------------------------------------"
echo "Phase 1 - This does the first half of the installation process for all of the support code:"
echo " - Performs the standard Ubuntu updates and installs the required programs/libraries"
echo " - Installs the ZED SDK"
echo " - Installs ROS"
echo " - Reboots.  Phase 2 must be run after Phase 1 has been completed"
echo
echo "Phase 2 - This phase finishes the installation process"
echo " - Installs Tensorflow"
echo " - Installs the Tensorflow model zoo"
echo " - Downloads the required Tensorflow models"
echo " - Clones the require support libraries"
echo " - Finishes the installation"
echo "-------------------------------------------------------------------------------------------"
echo $n "Select which phase (1 or 2) to run: $c"

read -n 1 option
echo
#echo $option

if  [ "$option" == "1" ]; then

gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.desktop.session idle-delay 0

# set the power mode to 30W all cores
sudo nvpmodel -m 3

## ----------------------------------------------------------------------------
# add the main repos 
sudo apt-add-repository universe
sudo apt-add-repository multiverse
sudo apt-add-repository restricted

sudo apt-get update -y
sudo apt-get upgrade -y

sudo apt-get install -y lsb-release wget less udev apt-transport-https nano cmake cmake-gui usbutils git build-essential
sudo apt-get install -y libusb-dev curl putty software-properties-common iputils-ping zip gfortran
sudo apt-get install -y libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev libjpeg8-dev liblapack-dev libblas-dev 
sudo apt-get install -y libxml2-dev libxslt-dev net-tools

## ----------------------------------------------------------------------------
## Get the ZED SDK - ZED_SDK_Tegra_JP43_v3.1.2.run
wget https://stereolabs.sfo2.digitaloceanspaces.com/zedsdk/3.1/ZED_SDK_Tegra_JP43_v3.1.2.run
chmod +x ZED_SDK_Tegra_JP43_v3.1.2.run
sudo ./ZED_SDK_Tegra_JP43_v3.1.2.run silent
sudo chmod 755 -R /usr/local/zed
sudo chown -R jax:jax /usr/local/zed

#rm -rf /var/lib/apt/lists/*
sudo mkdir -p /root/Documents/ZED/
sudo usermod -a -G video $USER
sudo usermod -a -G dialout $USER

## ----------------------------------------------------------------------------
## Setup and install ROS
current_deb=$DEBIAN_FRONTEND
sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime
export DEBIAN_FRONTEND=noninteractive
sudo apt-get install -y tzdata
sudo dpkg-reconfigure --frontend noninteractive tzdata

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update -y

sudo apt-get install -y ros-melodic-desktop-full 
sudo apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool

sudo rosdep init
sudo rosdep fix-permissions
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> /home/$USER/.bashrc
echo "export PLATFORM=JAX" >> /home/$USER/.bashrc

export DEBIAN_FRONTEND=$current_deb

source /home/$USER/.bashrc

# add a faux user name and email to git to push and pull changes
git config --global user.name "jax"
git config --global user.email "jax@example.com"

sudo reboot
exit 1

elif [ "$option" == "2" ]; then

## ----------------------------------------------------------------------------
## Install some python packages
curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py"
python get-pip.py

pip install -U tensorflow_hub numpy launchpadlib pandas Cython contextlib2 pillow lxml jupyter matplotlib utils --user
#/home/$USER/.local/bin/pip install tensorflow-gpu==1.14 tensorflow_hub --user --ignore-installed
#wget https://developer.download.nvidia.com/compute/redist/jp/v411/tensorflow-gpu/

pip install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v411/ tensorflow-gpu==1.13.0rc0+nv19.2 --user
#wget https://developer.download.nvidia.com/compute/redist/jp/v411/tensorflow-gpu/tensorflow_gpu-1.13.0rc0+nv19.2-cp27-cp27mu-linux_aarch64.whl
#/home/$USER/.local/bin/pip install tensorflow_gpu-1.13.0rc0+nv19.2-cp27-cp27mu-linux_aarch64.whl --user


## ----------------------------------------------------------------------------
## get the required ROS wrapper packages and the required support repos

cd /home/$USER
mkdir -p /home/$USER/catkin_ws
mkdir -p /home/$USER/catkin_ws/src
mkdir -p /home/$USER/Projects

wget http://dlib.net/files/dlib-19.19.tar.bz2
tar -xf dlib-19.19.tar.bz2
rm dlib-19.19.tar.bz2

git clone -b 'v3.0.3' --single-branch https://github.com/stereolabs/zed-ros-wrapper.git /home/$USER/catkin_ws/src/zed-ros-wrapper
git clone https://github.com/davemers0160/robot.git /home/$USER/catkin_ws/src/robot
git clone https://github.com/davemers0160/Common.git /home/$USER/Projects/Common


## ----------------------------------------------------------------------------
# source /opt/ros/melodic/setup.bash
cd /home/$USER/catkin_ws
catkin_make -j6


## ----------------------------------------------------------------------------
## get a couple of object detection models
cd /home/$USER
git clone https://github.com/tensorflow/models /home/$USER/models

cd /home/$USER/models/research
#wget -O protobuf.zip https://github.com/google/protobuf/releases/download/v3.0.0/protoc-3.0.0-linux-x86_64.zip
wget -O protobuf.zip https://github.com/protocolbuffers/protobuf/releases/download/v3.7.0/protoc-3.7.0-linux-aarch_64.zip
unzip protobuf.zip 
./bin/protoc object_detection/protos/*.proto --python_out=.

cd /home/$USER
#wget http://download.tensorflow.org/models/object_detection/ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03.tar.gz
#tar -xzvf ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03.tar.gz
#mv ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03 ssd_resnet50_v1_fpn
#rm ssd_resnet50_v1_fpn_shared_box_predictor_640x640_coco14_sync_2018_07_03.tar.gz

wget http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v2_oid_v4_2018_12_12.tar.gz
tar -xzvf ssd_mobilenet_v2_oid_v4_2018_12_12.tar.gz
mv ssd_mobilenet_v2_oid_v4_2018_12_12 ssd_mobilenet_v2_oid
rm ssd_mobilenet_v2_oid_v4_2018_12_12.tar.gz
    
wget http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12.tar.gz
tar -xzvf faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12.tar.gz
mv faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12 faster_rcnn_inception_resnet_v2_oid
rm faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12.tar.gz

wget http://download.tensorflow.org/models/object_detection/ssd_resnet101_v1_fpn_shared_box_predictor_oid_512x512_sync_2019_01_20.tar.gz
tar -xzvf ssd_resnet101_v1_fpn_shared_box_predictor_oid_512x512_sync_2019_01_20.tar.gz
mv ssd_resnet101_v1_fpn_shared_box_predictor_oid_512x512_sync_2019_01_20 ssd_resnet101_v1_fpn_oid
rm ssd_resnet101_v1_fpn_shared_box_predictor_oid_512x512_sync_2019_01_20.tar.gz

## ----------------------------------------------------------------------------
## put the everything in the right paths
echo "export PYTHONPATH=$PYTHONPATH:/home/$USER/models:/home/$USER/models/research:/home/$USER/models/research/slim:/home/$USER/models/research/object_detection" >> /home/$USER/.bashrc
echo "PATH=$PATH:/home/$USER/.local/bin" >> /home/$USER/.bashrc
echo "source /home/$USER/catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc
echo "export ROS_IP=$(hostname -I)" >> /home/$USER/.bashrc

fi

echo "Installation complete!"
echo

echo "Run the following command:"
echo "sudo nano /boot/extlinux/extlinux.conf"
echo "- add the following the the APPEND line: \" usbcore.usbfs_memory_mb=2000 \" "

# check: cat /sys/module/usbcore/parameters/usbfs_memory_mb


