#! /bin/bash

URI=""
IP=""

MASTER=""
HOST=""
CONFIG=""

if [ "$1" ]; then
  MASTER="$1"
fi

if [ "$2" ]; then
  HOST="$2"
fi

if [ "$2" ]; then
  CONFIG="$3"
fi


if [ "$CONFIG" == "wan" ]; then
  # MASTER
  if [ "$MASTER" == "jackal" ]; then
    URI="111.70.9.27"

  elif [ "$MASTER" == "husky" ]; then
    URI="111.70.9.53"
  fi

  # HOST
  if [ "$HOST" == "jackal" ]; then
    IP="111.70.9.27"

  elif [ "$HOST" == "husky" ]; then
    IP="111.70.9.53"

  elif [ "$HOST" == "pc" ]; then
    IP=$(/sbin/ip -o -4 addr list wlo1 | awk '{print $4}' | cut -d/ -f1)
  fi
fi

if [ "$CONFIG" == "lan" ]; then
  # MASTER
  if [ "$MASTER" == "jackal" ]; then
    URI="192.168.0.100"

  elif [ "$MASTER" == "husky" ]; then
    URI="192.168.0.104"
  fi

  # HOST
  if [ "$HOST" == "jackal" ]; then
    IP="192.168.0.100"

  elif [ "$HOST" == "husky" ]; then
    IP="192.168.0.104"

  elif [ "$HOST" == "pc" ]; then
    IP=$(/sbin/ip -o -4 addr list wlo1 | awk '{print $4}' | cut -d/ -f1)
  fi
fi

if [ -z "$URI" ]; then
  echo "ROS MASTER NOT SET. DEFAULTS TO 127.0.0.1"
  URI=127.0.0.1
fi

if [ -z "$IP" ]; then
  echo "ROS IP NOT SET. DEFAULTS TO 127.0.0.1"
  IP=127.0.0.1
fi

export ROS_MASTER_URI="http://${URI}:11311"
export ROS_IP=$IP

echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "ROS_IP=${ROS_IP}"

if [ -f /opt/ros/melodic/setup.bash ]; then
  echo "source: /opt/ros/melodic/setup.bash"
  source /opt/ros/melodic/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
  echo "source: /opt/ros/noetic/setup.bash"
  source /opt/ros/noetic/setup.bash
fi

if [ -f ./catkin_ws/devel/setup.bash ]; then
  echo "./catkin_ws/devel/setup.bash"
  source ./catkin_ws/devel/setup.bash
fi
