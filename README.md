# mocapnet_rosnode
A ROS node for the MocapNET 3D Pose Estimator

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
