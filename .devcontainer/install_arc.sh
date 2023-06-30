#!/bin/bash

# copy and install arc
echo "install arc from src" 
sudo cp -rf /workspaces/ros2_ws/lib/arc /opt/arc/ 
cd /opt/arc/lib
sudo mkdir build
cd /opt/arc/lib/build 
sudo cmake .. && sudo make -j$(nproc) install 

# install python packages
cd /opt/arc/python
sudo pip install -r requirements.txt
sudo pip install ./arc_core
sudo pip install ./arcpy

echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
source ~/.bashrc

cd /opt/arc/mujoco
sudo pip install -r requirements.txt

