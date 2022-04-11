# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:foxy-ros-base-focal

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    openssh-server \
    iputils-ping \
    wget \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-joy \
    ros-foxy-image-tools \
    && rm -rf /var/lib/apt/lists/*

# install pip
RUN wget -q -O - https://bootstrap.pypa.io/get-pip.py | python3
# use pip to install pysabertooth
RUN python3 -m pip install pysabertooth


