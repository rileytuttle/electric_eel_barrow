#!/bin/bash

# need -it to run interactely
# --privileged is to pass devices from host to docker container (not sure if this works though
# --network=host to use host network interface. specifically to send ros topics back to remote pc on same network
# --rm so that we remove container on exit
# --init allows for exiting by killing PID 1
# --name set name
# --device=/dev/input/js0 passes gamepad from host to docker container
# --device=/dev/video0 passes camera from host to docker container
# --mount mounts the repo to the docker container
sudo docker run              \
    -it                      \
    --privileged             \
    --network=host           \
    --rm                     \
    --init                   \
    --name="ez_cart_builder" \
    --device=/dev/input/js0  \
    --device=/dev/video0     \
    --mount type=bind,source=/home/pi/electric_eel_barrow,target=/electric_eel_barrow ezcart_docker_image:latest
