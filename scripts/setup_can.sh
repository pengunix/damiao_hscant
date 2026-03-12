#!/bin/bash
sudo ip link set up can0
sudo ip link set up can1
sudo ip link set up can2
sudo ip link set up can3
sudo ip link set can0 type can bitrate 1000000 
sudo ip link set can1 type can bitrate 1000000 
sudo ip link set can2 type can bitrate 1000000 
sudo ip link set can3 type can bitrate 1000000 
sudo ip link set up can0
sudo ip link set up can1
sudo ip link set up can2
sudo ip link set up can3

roslaunch motor_ros motor_node.launch &
wait
exit 0