#!/bin/bash

sudo docker create --privileged --network=host --init --name="ez_cart_docker" --device/dev/input/js0 --device=/dev/video0 --mount type=bind,source=/home/pi/electric_eel_barrow,target=/electric_eel_barrow ezcart_docker_image:latest /electric_eel_barrow/start_ez_cart_ros.sh
