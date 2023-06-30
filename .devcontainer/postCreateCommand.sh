#!/bin/bash
source /opt/ros/humble/setup.bash
git lfs install
# for aas dependencies, see README.md
curl http://vac-as-apt.arc.local/apt/debian/as-apt@ait.ac.at.gpg.key | sudo tee /etc/apt/trusted.gpg.d/as-apt@ait.ac.at.asc
echo "deb http://vac-as-apt.arc.local/apt/debian/ros/dev $(lsb_release -cs) main" | sudo tee -a /etc/apt/sources.list.d/ait-vac-as.list
echo "yaml http://vac-as-apt.arc.local/apt/debian/ros/dev/rosdistro/ait-rosdeps.yaml" | sudo tee -a /etc/ros/rosdep/sources.list.d/ait.list
echo "yaml http://vac-as-apt.arc.local/apt/debian/ros/dev/rosdistro/ait-rosdeps-$(dpkg --print-architecture).yaml" | sudo tee -a /etc/ros/rosdep/sources.list.d/ait.list
ROSDISTRO_INDEX_URL=http://vac-as-apt.arc.local/apt/debian/ros/dev/rosdistro/index.yaml rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
# --
source /opt/ros/humble/setup.bash
# import repositories
vcs import src < ros.repos
# update dependencies
rosdep install -riy --skip-keys "plotjuggler plotjuggler-ros plotjuggler-msgs ros2_controllers admittance_controller diff_drive_controller tricycle_controller gazebo_ros gazebo_ros2_control gazebo_ros_vision gazebo_grasp_plugin_ros testsite_description" --from-paths src


# copy and install arc
# ./install_arc.sh
./.devcontainer/install_arc.sh

