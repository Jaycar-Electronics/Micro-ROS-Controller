#!/bin/bash

function cont(){
  read -n 1 -s -r -p "Press any key to continue or Ctrl-C to cancel."
}
function install_ROS(){
  if [ `locale | grep UTF-8 | wc -l` -lt 1 ]; then
    sudo apt update && sudo apt install locales
    sudo locale-gen en_AU en_AU.UTF-8
    sudo update-locale LC_ALL=en_AU.UTF-8 LANG=en_AU.UTF-8
    export LANG=en_AU.UTF-8
  fi;

  sudo apt update && sudo apt install curl gnupg2 lsb-release -y
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

  sudo apt update
  sudo apt install ros-foxy-desktop -y
}

function install_python_depends(){
  source /opt/ros/foxy/setup.bash
  sudo apt install -y python3-pip
  pip3 install -U argcomplete flask eventlet flask_socketio
}

echo "========================================================"
echo "   ROS2 setup script for Micro ROS Controller: Jaycar"
echo "========================================================"
echo "This script will set up ROS2 on your system and install"
echo "required items to get the MRC software to work."
echo "This installs system level packages that alter the state"
echo "of your system. You can choose to do this in a VM or on "
echo "your own system if you want to."
echo ""
echo "Note that this will only work for ubuntu 20.04 derivatives and earlier"
echo "(20.10 is not yet supported in ROS2.)"
echo "while not attempted, raspbian is usually slow, and so should be ok."

echo "if you encounter any issues, hit up the ros2 link:"
echo "https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/"
echo "or the github issues tab"
echo ""
cont()

install_ROS()
install_python_depends()

echo "========================================================"
echo "                      All done!"
echo "========================================================"
