#!/bin/bash

./setup_bluetooth_connection.sh

source ez_cart_ros/install/setup.bash

ros2 launch ez_cart_ros/launch/ez_cart_launch.py
