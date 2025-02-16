# nao_virtual_noetic
This repo is an updated version of [nao_virtual](https://github.com/ros-naoqi/nao_virtual) for ROS Noetic (no changes to this package in particular, but hopefully better instructions)

## Dependencies:

To run you need another package in your workspace:
- nao_robot

## Installation (based on [awesomebytes](https://github.com/awesomebytes) [pepper_virtual description](https://github.com/awesomebytes/pepper_virtual)):

```
mkdir -p nao_sim_ws/src
cd nao_sim_ws/src
git clone https://github.com/dcuevasa/nao_virtual_noetic
git clone TODO
git clone TODO
# In case you are missing any of these
sudo apt-get install ros-noetic-tf2-sensor-msgs ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins ros-noetic-controller-manager ros-noetic-ddynamic-reconfigure-python
cd ..
catkin_make
source devel/setup.bash
# Launch your preferred simulation here
roslaunch nao_gazebo_plugin nao_gazebo.launch
```
