#!/bin/bash

sudo docker create --privileged --name="ez_cart_docker" --device/dev/input/js0 --mount type=bind,source=/home/pi/electric_eel_barrow,target=/electric_eel_barrow ros_docker:latest /electric_eel_barrow/start_ez_cart_ros.sh
