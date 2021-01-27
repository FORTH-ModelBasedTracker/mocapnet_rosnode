#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"


if [ -f dependencies/MocapNET/update.sh ]; then
cd dependencies/MocapNET/
git pull origin master
cd "$DIR"
else
echo "Could not find MocapNET, please rerun the initialize.sh script .." 
fi


#Now sync rest of code
cd "$DIR"
git pull origin main
  

exit 0
