#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

ORIG_DIR=`pwd`
 
cd "$DIR" 


#Simple dependency checker that will apt-get stuff if something is missing
# sudo apt-get install git
SYSTEM_DEPENDENCIES="git"
#------------------------------------------------------------------------------
for REQUIRED_PKG in $SYSTEM_DEPENDENCIES
do
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
echo "Checking for $REQUIRED_PKG: $PKG_OK"
if [ "" = "$PKG_OK" ]; then

  echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."

  #If this is uncommented then only packages that are missing will get prompted..
  #sudo apt-get --yes install $REQUIRED_PKG

  #if this is uncommented then if one package is missing then all missing packages are immediately installed..
  sudo apt-get install $SYSTEM_DEPENDENCIES  
  break
fi
done
#------------------------------------------------------------------------------




if [ -f dependencies/MocapNET/README.md ]; then
echo "MocapNET appears to already exist .."
else 
  echo "     Do you want to automatically download the MocapNET project ? " 
  echo " If you don't have it already installed somewhere on your hard drive select yes " 
  echo
  echo -n " (Y/N)?"
  read answer
  if test "$answer" != "N" -a "$answer" != "n";
  then 
     echo "Downloading.."
     cd "$DIR/dependencies"
     #git clone https://github.com/FORTH-ModelBasedTracker/MocapNET
     #cd MocapNET
     #./initialize.sh
     cd "$DIR"
   else
   echo "Please give the exact path to your MocapNET installation!"
   echo "Be careful with spaces!"
   echo -n "Path:"
   read MOCAPNET_PATH
    if [ -f $MOCAPNET_PATH/initialize.sh ]; then
     echo "Thank you, found it..!"
     
     cd "$DIR/dependencies"
     ln -s $MOCAPNET_PATH MocapNET
    fi
  fi
fi 




if [ -f dependencies/MocapNET/initialize.sh ]; then
 echo "Setting up links"
 cd "$DIR/bin"
 #make sure stuff are in place..
 ln -s ../dependencies/MocapNET/dataset/
 ln -s ../dependencies/MocapNET/libJointEstimator2D.so
 ln -s ../dependencies/MocapNET/libMocapNETLib2.so

fi


exit 0
