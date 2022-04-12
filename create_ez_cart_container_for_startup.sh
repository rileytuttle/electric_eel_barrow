#!/bin/bash

# --privileged is to pass devices from host to docker container (not sure if this works though
# --restart unless-stopped will restart when start_ez_cart_ros doesnt detect gamepad and kills PID 1
# --network=host to use host network interface. specifically to send ros topics back to remote pc on same network
# --rm so that we remove container on exit
# --init allows for exiting by killing PID 1
# --name set name
# --device=/dev/input/js0 passes gamepad from host to docker container
# --device=/dev/video0 passes camera from host to docker container
# --mount mounts the repo to the docker container
sudo docker create           \
    --privileged             \
    --restart unless-stopped \
    --network=host           \
    --init                   \
    --name="ez_cart_docker"  \
    --device=/dev/input/js0  \
    --device=/dev/video0     \
    --mount type=bind,source=/home/pi/electric_eel_barrow,target=/electric_eel_barrow ezcart_docker_image:latest /electric_eel_barrow/start_ez_cart_ros.sh
