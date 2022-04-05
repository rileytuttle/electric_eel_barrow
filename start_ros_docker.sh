#!/bin/bash

sudo killall -9 bluetoothd
sudo docker run -it --mount type=bind,source="$(pwd)",target=/electric_eel_barrow --privileged --net=host --name="ez_cart_docker" ros_docker_test:latest
