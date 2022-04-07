#!/bin/bash

/electric_eel_barrow/setup_bluetooth_connection.sh

source /electric_eel_barrow/ez_cart_ros/install/setup.bash

ros2 launch /electric_eel_barrow/ez_cart_ros/launch/ez_cart_launch.py
