# mocapnet_rosnode

A ROS node for the [MocapNET 3D Pose Estimator](https://github.com/FORTH-ModelBasedTracker/MocapNET) 


![mocapnet_rosnode screenshot with rviz](https://raw.githubusercontent.com/FORTH-ModelBasedTracker/mocapnet_rosnode/main/doc/screenshot.jpg)


Please only use this issues page for issues with the ROS wrapper, the main development repository of MocapNET is [here](https://github.com/FORTH-ModelBasedTracker/MocapNET) 


__Step 0__ : [Install ROS](http://wiki.ros.org/Installation/) 

__Step 1__ : [Create your Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

__Step 2__ :
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

__Step 3__ : 
```
cd your/work/space/path/here
source devel/setup.bash
roslaunch mocapnet_rosnode mocapnet_rosnode.launch
```

Feel free to make your own ros launcher by using the default [mocapnet_rosnode.launch](https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode/blob/main/launch/mocapnet_rosnode.launch) as a template to use your cameras and TF Root.

The default settings are `/camera/rgb/image_rect_color` for the RGB image topic, `/camera/rgb/camera_info` for the camera calibration and map as the TF root. 

You can download [this sample rosbag](https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode/raw/main/doc/sample.bag) to take a look on the TF tree by using rosbag play doc/sample.bag --loop 

__Step 4 (Optional)__: If you don't have your own ROS package to acquire RGB input you can use the bundled camera acquisition software by executing the following instructions
```
cd your/work/space/path/here
source devel/setup.bash
src/mocapnet_rosnode/scripts/createAcquisition.sh
catkin_make
```

This will create a link to the [rgbd_acquisition ROSNode](https://github.com/AmmarkoV/RGBDAcquisition/tree/master/3dparty/ROS/rgbd_acquisition) in your workspace and allow you to stream your webcam (/dev/video0) using

```
roslaunch rgbd_acquisition rgb_acquisition.launch moduleID:=V4L2 deviceID:=/dev/video0 width:=640 height:=480 framerate:=30
```

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

If you want immediate access to the euler angles of the BVH frame internally populated by MocapNET you can access them using the rostopic `/mocapnet_rosnode/bvhFrame`

Running: 

```
rostopic echo /mocapnet_rosnode/bvhFrame 

```

Will yield 498 element arrays encoding the full 3D human skeleton, to interpret the vectors in your program you can use the [enum MOCAPNET_Output_Joints](https://github.com/FORTH-ModelBasedTracker/MocapNET/blob/master/src/MocapNET2/MocapNETLib2/mocapnet2.hpp#L1572), for example [element 318](https://github.com/FORTH-ModelBasedTracker/MocapNET/blob/master/src/MocapNET2/MocapNETLib2/mocapnet2.hpp#L1892) encodes left elbow z rotation as understood by the label `MOCAPNET_OUTPUT_LELBOW_ZROTATION,//318`. Copying the whole vector as a motion record on [this bvh file](https://github.com/FORTH-ModelBasedTracker/MocapNET/blob/master/dataset/headerWithHeadAndOneMotion.bvh#L1022) yields a valid BVH file.

```
data: [-9.512088775634766, -4.7546868324279785, -288.34246826171875, 27.72639274597168, -0.3889874815940857, -20.50408363342285, 0.0, 0.0, 0.0, 15.126297950744629, 12.259260177612305, 15.964614868164062, -5.329497337341309, 5.251098155975342, -7.2560014724731445, 0.0, 0.0, 0.0, 2.8856470584869385, -32.6014404296875, -9.461994171142578, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 63.49156188964844, 32.619571685791016, 7.534119606018066, -12.211994171142578, -9.794024467468262, 22.890546798706055, -3.2917959690093994, -4.926611423492432, -5.74922513961792, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -81.3495101928711, 69.02851867675781, 1.6789360046386719, 18.163463592529297, -16.729158401489258, -33.58930587768555, 3.7542624473571777, 3.836533308029175, 0.9339077472686768, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.606363296508789, -10.068902969360352, 7.630038738250732, -1.881280541419983, 3.910395860671997, -6.197346351655142e-07, -5.114427089691162, -9.173490524291992, -10.093374252319336, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.7712984085083, -3.146512269973755, -25.90770721435547, 7.954533100128174, 27.014575958251953, 2.2015790079876751e-07, 1.4231140613555908, -8.588176727294922, 4.44589376449585, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```


By default the 2D joint estimator used is `forth` and is best suited for realtime execution and decent 2D pose estimation accuracy. If you have a beefy GPU you can change the ["joint2DEstimator" launch parameter](https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode/blob/main/launch/mocapnet_rosnode.launch#L18) to `vnect` or `openpose` for higher accuracy at slower speeds.

Please note that camera movement is "encoded" as skeleton movement so usage of a static camera should be ok, however if using a moving camera you should [disable the publishCameraTF ROS parameter](https://github.com/FORTH-ModelBasedTracker/mocapnet_rosnode/blob/main/launch/mocapnet_rosnode.launch#L10) and publish your own "mocapnetCamera" tf2:transform.

Good luck!
