#!/bin/bash

sudo docker run -it --privileged --network=host --rm --init --name="ez_cart_builder" --device=/dev/input/js0 --device=/dev/video0 --mount type=bind,source=/home/pi/electric_eel_barrow,target=/electric_eel_barrow ezcart_docker_image:latest
