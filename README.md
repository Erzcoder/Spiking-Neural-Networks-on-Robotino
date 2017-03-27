# Spiking-Neural-Networks-on-Robotino

This repository contains code to control FESTO's mobile robot Robotino with spiking neural networks. 
If no physical robot is available, one can instead control the virtual Turtlebot inside the Gazebo simulation environment. 

This Repository consists of the following packages:
- cv_camera:               interface between a camera and ROS
- dvs_simulator:           difference of camera images to estimate motion flow
- edvstools:               package for interfacing with the DVS sensor
- experimental_scripts:    scripts that we used for testing before adding them to the system
- gazebo models:           models that we use in Gazebo
- image-preprocessing:     takes images and preprocesses them
- neural_network:          ROS nodes that run a spiking neural network
- spinnaker_ros:           interface between SpiNNaker and ROS
- Util:                    our encoding spript to send images as several Int64 messages to SpiNNaker
    


## Prerequisits:
Due to the used tools, this project can only be run on Linux machines. 
### Install ROS
Use http://wiki.ros.org/kinetic/Installation/Ubuntu
### Install NEST
Use http://www.nest-initiative.org/
### Install PyNN
Use http://neuralensemble.org/PyNN/
### Install sPyNNaker
http://spinnakermanchester.github.io/2015.005.Arbitrary/PyNNOnSpinnakerInstall.html
### Set up the (ROS) interface to SpiNNaker
Use https://github.com/reiths/ros_spinnaker_interface


 
