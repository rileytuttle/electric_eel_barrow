currently the raspberry pi is

pi@raspberrypi.local
password: "rileytuttle"


run connect_ds4_controller after getting controller into pairing mode

then make sure the docker image is running. it should be

to check the state of running dockers
sudo docker ps -a

to restart the docker
sudo systemctl restart ez_cart.service

may have to connect to ros docker and start talker for some reason

./connect_ros_docker.sh
cd /electric_eel_barrow/ez_cart_ros
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run ez_cart talker
