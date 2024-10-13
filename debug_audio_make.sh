#!/bin/bash
__doc__="
Ensure
sudo apt-get install libcapstone-dev
"

set -e

echo "============================"
echo "Attempting to make audiofile"
echo "============================"
cd ~/code/sm64-random-assets/tpl/sm64-port/tools/audiofile
VERBOSE=1 make

#echo "Attempting to make tabledesign"
#cd ~/code/sm64-random-assets/tpl/sm64-port/tools/sdk-tools/tabledesign
#VERBOSE=1 make


echo "============================"
echo "Attempting to make aiff_extract_codebook"
echo "============================"
cd ~/code/sm64-random-assets/tpl/sm64-port/tools
VERBOSE=1 make

echo "done with debug audio make"
