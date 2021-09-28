#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

cd ..
MOCPANET_ROSNODE=`pwd`
cd ..
ROS_WORKPACE=`pwd`
ln -s mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition/3dparty/ROS/rgbd_acquisition
cd rgbd_acquisition/src
ln -s $ROS_WORKPACE/mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition/acquisition/Acquisition.h
ln -s $ROS_WORKPACE/mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition/tools/Calibration/calibration.h

exit 0
