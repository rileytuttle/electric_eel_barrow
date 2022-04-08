#!/bin/bash

if [[ -e /dev/input/js0 ]]
then
    echo "controller connected"
else
    echo "controller not connected exiting docker to start up again"
    kill 1
fi

source /electric_eel_barrow/ez_cart_ros/install/setup.bash

ros2 launch /electric_eel_barrow/ez_cart_ros/launch/ez_cart_launch.py
