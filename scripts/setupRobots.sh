#!/bin/bash


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

cd ..
cd ..
ROS_WORKSPACE=`pwd`
ROS_VERSION=`rosversion -d`

sudo apt-get install ros-$ROS_VERSION-pinocchio vulkan-utils

cd ~
mkdir -p Documents/3dParty
cd Documents/3dParty

#Install QPMAD
git clone https://github.com/asherikov/qpmad
cd qpmad
git pull
mkdir build 
cd build
cmake ..
sudo make install 
cd ..
cd ..

#Install RAISIM
git clone https://github.com/raisimTech/raisimUnityOpengl
git clone https://github.com/raisimTech/raisimLib
cd raisimLib
RAISIM_LIB_PATH=`pwd`
cd ..
RAISIM_BUILD_PATH=`pwd`


echo "Please add your RAISIM License on ~/.raisim as activation.raisim"

export WORKSPACE=$RAISIM_BUILD_PATH
export LOCAL_INSTALL=$RAISIM_BUILD_PATH/raisim_build

cd $RAISIM_LIB_PATH
mkdir build
cd build

cmake .. -DCMAKE_INSTALL_PREFIX=$LOCAL_INSTALL -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)")

make -j4
make install

echo "Please add ot your ~/.bashrc"
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$RAISIM_BUILD_PATH/raisim_build/lib"
echo "export PYTHONPATH=\$PYTHONPATH:$RAISIM_BUILD_PATH/raisim_build/lib"


cd $ROS_WORKSPACE
https://github.com/mrsp/whole_body_control_action_client_test
chmod +x whole_body_control_action_client_test/src/pose_control.py
chmod +x whole_body_control_action_client_test/src/walking_control.py

git clone https://github.com/mrsp/whole_body_ik
git clone https://github.com/mrsp/whole_body_ik_msgs
git clone https://github.com/mrsp/raisim_ros


exit 0
