#!/bin/bash

/electric_eel_barrow/setup_bluetooth_connection.sh
/electric_eel_barrow/ds4_connection_manager.sh

sleep 5 #needs this otherwise the joy node doesn't talk to the bluetooth controller

source /electric_eel_barrow/ez_cart_ros/install/setup.bash

ros2 launch /electric_eel_barrow/ez_cart_ros/launch/ez_cart_launch.py
