#!/bin/bash


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

cd ..
cd ..

source ../devel/setup.bash

screen -S CORE -d -m roscore
screen -S RAISIM -d -m roslaunch raisim_ros nao.launch
screen -S IK -d -m roslaunch whole_body_ik nao_wbc.launch
screen -S ACTIONCLIENT -d -m rosrun  action_client_test pose_control.py
 
#screen -S CAMERA -d -m roslaunch rgbd_acquisition rgb_acquisition.launch moduleID:=TEMPLATE deviceID:=sven.mp4-data width:=1920 height:=1080 framerate:=30
screen -S CAMERA -d -m roslaunch rgbd_acquisition rgb_acquisition.launch moduleID:=V4L2 deviceID:=/dev/video0 width:=640 height:=480 framerate:=30
screen -S MOCAPNET -d -m roslaunch mocapnet_rosnode mocapnet_rosnode.launch


screen -list











exit 0
