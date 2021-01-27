# mocapnet_rosnode

A ROS node for the MocapNET 3D Pose Estimator
Please only use this issues page for issues with the ROS wrapper, the main development repository of MocapNET is [here](https://github.com/FORTH-ModelBasedTracker/MocapNET) 


Step 0 : [Install ROS](http://wiki.ros.org/Installation/) 

Step 1 : Create your Workspace ([info on how to do this here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment))

Step 2 :
```
cd your/work/space/path/here
cd src/
git clone https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode
cd mocapnet_rosnode/
./initialize.sh
cd ../../build
cmake ..
make
```

Step 3 : 
```
cd your/work/space/path/here
source devel/setup.bash
roslaunch mocapnet_rosnode mocapnet_rosnode.launch
```

Feel free to make your own ros launcher by using the default [mocapnet_rosnode.launch](https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode/blob/main/launch/mocapnet_rosnode.launch) as a template to use your cameras and TF Root.

The default settings are /camera/rgb/image_rect_color for the RGB image topic, /camera/rgb/camera_info for the camera calibration and map as the TF root. 

Good luck!
