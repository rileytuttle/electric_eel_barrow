#! /bin/sh
# chkconfig: 345 99 10

case "$1" in
    start)
        sudo bash /electric_eel_barrow/start_ros_docker_ezcart.sh
        ;;
    *)
        ;;
esac
exit 0
