# Spiking-Neural-Networks-on-Robotino

This repository contains code to control FESTO's mobile robot Robotino with spiking neural networks. 


## 1. Control the Robotino with ROS
Using Expertinos' ROS packages (for ROS Indigo Distro), forked at https://github.com/Erzcoder/ros-indigo-robotino

The topic should be `cmd_vel/odom` 

Command line usage: `rostopic pub -1 cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.5}}`

### One might also try out teleoperation with the keyboard: 
`sudo apt-get install ros-indigo-teleop-twist-keyboard`

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## 2. Run spiking neural network models on SpiNNaker & build suitable SNN architectures


## 3. Set up the (ROS) interface to SpiNNaker

Use https://github.com/reiths/ros_spinnaker_interface

### Transform the pixel data from the camera to spikes and feed them into the network

### Transform the spikes from the output neurons to steering commands



## Tasks

- Robotino should be able to follow an arbitrary lane by only using vision data which is fed into a SNN. 

## Optional additional Tasks

- Use a dynamic vision sensor (DVS) instead of a camera
- add collision avoidance
- add global path planning?
- run everything inside gazebo -> co-simulation?

## Questions and Ideas: 

  - run the spiking neural networks locally on a computer using NEST as simulator and send the results via costum ROS msgs (and later transition to SpiNNaker to run more complex networks)?
 
