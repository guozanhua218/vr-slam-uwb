#! /bin/bash

URI=127.0.0.1
IP=127.0.0.1

if [ "$1" ]; then

  echo "$1"
  URI=127.0.0.1
  IP=127.0.0.1

  if [ "$1" == "wan" ]; then
    URI=111.70.9.27
    IP=111.70.9.27

  elif [ "$1" == "lan" ]; then
    URI=192.168.0.100
    IP=192.168.0.100
    
  else

    if [ "$2" ]; then
    URI=$1
    IP=$2
    fi

  fi

fi

export ROS_MASTER_URI="http://${URI}:11311"
export ROS_IP=$IP

echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "ROS_IP=${ROS_IP}"

source /opt/ros/melodic/setup.bash
source catkin_ws/devel/setup.bash
