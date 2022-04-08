#!/bin/bash

sudo docker run -it --privileged --name="ez_cart_docker" --device=/dev/input/js0 --mount type=bind,source=/home/pi/electric_eel_barrow,target=/electric_eel_barrow ros_docker:latest
