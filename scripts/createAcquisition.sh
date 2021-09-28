#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

cd ..
MOCPANET_ROSNODE=`pwd`
cd ..
ROS_WORKPACE=`pwd`
ln -s mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition/3dparty/ROS/rgbd_acquisition


#Special implementation of linkToRGBDAcquisitionLib.sh for this ROS Workspace 
#https://github.com/AmmarkoV/RGBDAcquisition/blob/master/3dparty/ROS/rgbd_acquisition/linkToRGBDAcquisitionLib.sh

cd rgbd_acquisition/src
ln -s $ROS_WORKPACE/mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition/acquisition/Acquisition.h
ln -s $ROS_WORKPACE/mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition/tools/Calibration/calibration.h
cd ..


cd bin
#Copy things 
ln -s $ROS_WORKPACE/acquisition/libAcquisition.so
ln -s $ROS_WORKPACE/tools/Calibration/libCalibrationLibrary.a
ln -s $ROS_WORKPACE/tools/OperatingSystem/libOperatingSystem.a
ln -s $ROS_WORKPACE/tools/Codecs/libCodecs.a
ln -s $ROS_WORKPACE/tools/Timers/libTimers.a
ln -s $ROS_WORKPACE/tools/LocationServices/libLocationServices.a
ln -s $ROS_WORKPACE/openni2_acquisition_shared_library/libOpenNI2Acquisition.so
ln -s $ROS_WORKPACE/template_acquisition_shared_library/libTemplateAcquisition.so
ln -s $ROS_WORKPACE/depthsense_acquisition_shared_library/libDepthSenseAcquisition.so 
ln -s $ROS_WORKPACE/v4l2_acquisition_shared_library/libV4L2Acquisition.so
ln -s $ROS_WORKPACE/v4l2stereo_acquisition_shared_library/libV4L2StereoAcquisition.so

ln -s $ROS_WORKPACE/libfreenect_acquisition_shared_library/libFreenectAcquisition.so
ln -s  /usr/local/lib/libfreenect_sync.so.0.5
ln -s  /usr/local/lib/libfreenect.so.0.5

ln -s $ROS_WORKPACE/librealsense_acquisition_shared_library/libRealsenseAcquisition.so
#ln -s  /usr/local/lib/libfreenect_sync.so.0.5
#ln -s  /usr/local/lib/libfreenect.so.0.5


ln -s $ROS_WORKPACE/editor/Editor
ln -s $ROS_WORKPACE/viewer/Viewer
cd ..



exit 0
