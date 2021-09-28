#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

cd ..
MOCPANET_ROSNODE=`pwd`
cd ..
ROS_WORKPACE=`pwd`
ln -s mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition/3dparty/ROS/rgbd_acquisition
exit 0
