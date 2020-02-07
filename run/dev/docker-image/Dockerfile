FROM osrf/ros:eloquent-desktop

MAINTAINER Eric Solomon

# setup gazebo keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup gazebo sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN apt-get update && apt-get install -q -y \
    libgazebo10-dev=10.1.0-1* \
    && rm -rf /var/lib/apt/lists/*
