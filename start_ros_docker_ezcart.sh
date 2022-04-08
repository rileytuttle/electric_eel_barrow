#!/bin/bash

sudo docker run --rm -it --mount type=bind,source="$(pwd)",target=/electric_eel_barrow --privileged --device=/dev/input/js0 --net=host --name="ez_cart_docker" ros_docker:latest electric_eel_barrow/start_ez_cart_ros.sh
