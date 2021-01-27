#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

ORIG_DIR=`pwd`
 
cd "$DIR" 

if [ -f dependencies/RGBDAcquisition/README.md ]; then
echo "MocapNET appears to already exist .."
else
 cd "$DIR/dependencies"
 git clone https://github.com/FORTH-ModelBasedTracker/MocapNET
 cd MocapNET
 ./initialize.sh
 cd "$DIR"
fi


exit 0
