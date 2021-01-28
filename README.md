# mocapnet_rosnode

A ROS node for the [MocapNET 3D Pose Estimator](https://github.com/FORTH-ModelBasedTracker/MocapNET) 


![mocapnet_rosnode screenshot with rviz](https://raw.githubusercontent.com/FORTH-ModelBasedTracker/mocapnet_rosnode/main/doc/screenshot.jpg)


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

You can control the camera position and roll pitch yaw to match your TF tree.
The default settings are the following :

```
rosservice call /mocapnet_rosnode/setCameraXPosition "value: 0.0" 
rosservice call /mocapnet_rosnode/setCameraYPosition "value: 0.0" 
rosservice call /mocapnet_rosnode/setCameraZPosition "value: 0.0" 
rosservice call /mocapnet_rosnode/setCameraRoll "value: 90.0" 
rosservice call /mocapnet_rosnode/setCameraPitch "value: 0.0" 
rosservice call /mocapnet_rosnode/setCameraYaw "value: 0.0" 

```

You can download [this sample rosbag](https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode/raw/main/doc/sample.bag) to take a look on the TF tree..

By default the 2D joint estimator used is "forth" and is best suited for realtime execution and decent 2D pose estimation accuracy. If you have a beefy GPU you can change the ["joint2DEstimator" launch parameter](https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode/blob/main/launch/mocapnet_rosnode.launch#L18) to "vnect" or "openpose" for higher accuracy at slower speeds.

Please note that camera movement is "encoded" as skeleton movement so usage of a static camera should be ok, however if using a moving camera you should [disable the publishCameraTF ROS parameter](https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode/blob/main/launch/mocapnet_rosnode.launch#L10) and publish your own "mocapnetCamera" tf2:transform.

Good luck!
