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

RGBD_ACQUISITION="$ROS_WORKPACE/mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition"

cd rgbd_acquisition/src
ln -s $RGBD_ACQUISITION/acquisition/Acquisition.h
ln -s $RGBD_ACQUISITION/tools/Calibration/calibration.h
cd ..


cd bin
#Copy things 
ln -s $RGBD_ACQUISITION/acquisition/libAcquisition.so
ln -s $RGBD_ACQUISITION/tools/Calibration/libCalibrationLibrary.a
ln -s $RGBD_ACQUISITION/tools/OperatingSystem/libOperatingSystem.a
ln -s $RGBD_ACQUISITION/tools/Codecs/libCodecs.a
ln -s $RGBD_ACQUISITION/tools/Timers/libTimers.a
ln -s $RGBD_ACQUISITION/tools/LocationServices/libLocationServices.a
ln -s $RGBD_ACQUISITION/openni2_acquisition_shared_library/libOpenNI2Acquisition.so
ln -s $RGBD_ACQUISITION/template_acquisition_shared_library/libTemplateAcquisition.so
ln -s $RGBD_ACQUISITION/depthsense_acquisition_shared_library/libDepthSenseAcquisition.so 
ln -s $RGBD_ACQUISITION/v4l2_acquisition_shared_library/libV4L2Acquisition.so
ln -s $RGBD_ACQUISITION/v4l2stereo_acquisition_shared_library/libV4L2StereoAcquisition.so

ln -s $RGBD_ACQUISITION/libfreenect_acquisition_shared_library/libFreenectAcquisition.so
ln -s  /usr/local/lib/libfreenect_sync.so.0.5
ln -s  /usr/local/lib/libfreenect.so.0.5

ln -s $RGBD_ACQUISITION/librealsense_acquisition_shared_library/libRealsenseAcquisition.so
#ln -s  /usr/local/lib/libfreenect_sync.so.0.5
#ln -s  /usr/local/lib/libfreenect.so.0.5


ln -s $ROS_WORKPACE/editor/Editor
ln -s $ROS_WORKPACE/viewer/Viewer
cd ..


cd $ROS_WORKPACE/mocapnet_rosnode/dependencies/MocapNET/dependencies/RGBDAcquisition/
mkdir build
cd build
cmake ..
make


exit 0
