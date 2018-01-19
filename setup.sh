#!/bin/bash

if [ $# -eq 0 ]
  then
    # No arguments supplied
    ubuntu_version_name="trusty"
    ros_version="indigo"

elif [ $# -eq 2 ]
  then
    ubuntu_version_name=${1}
    ros_version=${2}

else
  echo "ERROR: Wrong usage"
  echo "Usage: $0 ubuntu_code_name ros_code_name eg. $0 xenial kinetic"
  exit 1

fi

echo "Installing Dependencies for V4R (Ubuntu ${ubuntu_version_name} using ROS ${ros_version})..."
echo "If you want to change this you can pass in the codename of the Ubuntu release. Eg. $0 xenial kinetic for 16.04 with ROS kinetic"

if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu '${ubuntu_version_name}' main" > /etc/apt/sources.list.d/ros-latest.list'
fi

# V4R requires CMake ≥ 3.5.1, which is not packaged for Ubuntu Trusty
# We download official pre-built package and install it to /usr/local
if [[ ${ubuntu_version_name} == "trusty" ]]; then
  wget -q -O /tmp/cmake.sh https://cmake.org/files/v3.10/cmake-3.10.0-Linux-x86_64.sh
  sudo sh /tmp/cmake.sh --prefix=/usr/local --exclude-subdir --skip-license
else
  # On other Ubuntu versions (i.e. Xenial) we just install this package
  pkg='cmake'
fi

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq > /dev/null
sudo apt-get install -qq -y python-rosdep build-essential ${pkg} > /dev/null
sudo rosdep init > /dev/null

rosdep update > /dev/null
rosdep install -q --from-paths . -i -y -r --rosdistro ${ros_version} > /dev/null
