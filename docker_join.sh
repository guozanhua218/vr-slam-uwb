#!/usr/bin/env bash

IMG=sunfuchou/vr_slam_uwb:x86
REPOSITORY="sunfuchou/vr_slam_uwb"
TAG="x86"

IMG="${REPOSITORY}:${TAG}"

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}") && echo "$containerid"
docker exec -it \
    --privileged \
    -e DISPLAY="${DISPLAY}" \
    -e LINES="$(tput lines)" \
    "${containerid}" \
    bash
xhost -
