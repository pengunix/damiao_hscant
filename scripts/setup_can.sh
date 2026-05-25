#!/bin/bash
sudo ip link set down can0
sudo ip link set down can1
sudo ip link set down can2
sudo ip link set down can3
sudo ip link set can0 txqueuelen 100
sudo ip link set can1 txqueuelen 100
sudo ip link set can2 txqueuelen 100
sudo ip link set can3 txqueuelen 100
sudo ip link set can0 type can bitrate 500000 sample-point 0.8
sudo ip link set can1 type can bitrate 500000 sample-point 0.8
sudo ip link set can2 type can bitrate 500000 sample-point 0.8
sudo ip link set can3 type can bitrate 500000 sample-point 0.8
sudo ip link set up can0
sudo ip link set up can1
sudo ip link set up can2
sudo ip link set up can3

ros2 launch motor_ros motor_node.launch.py

sudo ip link set down can0
sudo ip link set down can1
sudo ip link set down can2
sudo ip link set down can3
exit 0
