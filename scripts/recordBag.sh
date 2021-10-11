#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

cd ..
rosbag record /tf /mocapnet_rosno/bvhFrame

exit 0
