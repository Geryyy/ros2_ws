# ROS2 Baustelle Workspace
This is the workspace for ROS2.

## Content
This workspace contains several nested repositories with the following packages


see the README files in the subfolders.

## Prerequisites
There are two ways to fullfill the requirements:
* If you have docker installed (described [here](.devcontainer/README.md) ) you can use the provided devcontainer with [visual studio code](#vscode)
* or use the following description for native installation:

Tested on Ubuntu 20.04 with ros2 humble, see the [ros documentation for installing ros2](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

### Ubuntu 20.04 focal
Because 20.04 is not a Tier 1 distribution any more, you have to build ros2 from source, see the [ros documentation for installing ros2 via source](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html), or update to Ubuntu 22.04 (see below).

After successful installation, manually source ros2 in every new bash window or, optionally, source it permanently by adding it to your `.bashrc` via
```bash
echo "source ~/ros2_humble/install/local_setup.bash" >> ~/.bashrc
```

To get upstream updates of ROS, read the [Maintaining a Source-Checkout](https://docs.ros.org/en/humble/Installation/Maintaining-a-Source-Checkout.html) manual.

### Ubuntu 22.04 jammy
Install ros2 via debian packages, see the [ros documentation for installing ros2 via debians](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
After successful installation, manually source ros2 in every new bash window or, optionally, source it permanently by adding it to your `.bashrc` via
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
### Common for all Ubuntu distros
Test if the provided examples run properly, e.g., run
```bash
ros2 run demo_nodes_cpp talker
```
and listen from another terminal window (source ros2 first)
```bash
ros2 topic echo chatter
```
If this does not work, there might be troubles with your network settings, probably coming from the Rudder Agent. Try some of the following:
* Disconnect your ethernet cable or change to WIFI
* Check your [multicast settings](https://docs.ros.org/en/galactic/How-To-Guides/Installation-Troubleshooting.html#enable-multicast)
* Try other network-related settings (use google)
```bash
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
sudo ifconfig lo multicast
```

If that worked, install rosdep and colcon with
```bash
sudo apt install python3-bloom python3-rosdep fakeroot python3-colcon-common-extensions
```
Initialize rosdep with
```bash
sudo rosdep init
```
and update it
```bash
rosdep update
```
Afterwards update your packagelist
```bash
sudo apt-get update
```
## Installation
All nested repositories for the workspace can be cloned using vcstool. 
First, you have to [store your git credentials](https://git-scm.com/docs/git-credential-store#_examples) 
globally to make vcstool work via
```bash
git config --global credential.helper store
git pull
```
Then, install vcstool with `sudo -H pip3 install vcstool` and run
```bash
vcs import src < ros.repos
```
from the current folder. This can take a while, depending on your internet speed.
Now all nested repositories should have been cloned successfully into the `src` folder.

### Ubuntu 20.04 focal
`rosdep` won't work with Ubuntu focal, because there are no releases for this ubuntu distro. Therefore, we have to manually solve the dependencies by downloading
```bash
cp ros_focal_humble.repos ~/ros2_humble/
cd ~/ros2_humble/
vcs import src < ros_focal_humble.repos
```
and installing via
```bash
colcon build --symlink-install --continue-on-error --packages-skip joystick_drivers ps3joy spacenav wiimote
```
Go back to this workspace and check if all dependencies were installed using rosdep
```bash
rosdep install -riy --from-paths src
```

### Ubuntu 22.04 jammy
Install all the dependencies using rosdep
```bash
rosdep install -riy --from-paths src
```

### Common for all Ubuntu distros
All remote dependencies should be installed. If not, you have to
manually install them, e.g., install armadillo via
```bash
sudo apt-get install libarmadillo-dev
```
After installing ros and resolving all prerequisites, you should now be able to build this workspace:
```bash
colcon build --symlink-install
```

## First steps
After it compiled successfully, source your workspace and run
```bash
. install/setup.bash
ros2 launch epsilon_crane_description description.launch.py gui:=True
```
If everything worked well, you should be able to see the crane in rviz.

If you haven't had installed gazebo before, rosdep should have done that. 
Permanently source gazebo by adding the following to your .bashrc via
```bash
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
```
and run
```bash
ros2 launch epsilon_crane_bringup gazebo_model.launch.py
```
to start the gazebo simulation.

See the [readme](src/epsilon_crane/epsilon_crane_bringup/README.md) of the nested package for more information.

## Work with the ros workspace
### Add another nested git repository
These packages are split into several individual git repositories for modularization and easier 
data exchange with other contributors. 
To add another git repository permanently, perform the following steps:
1. add it to [ros.repos](ros.repos) (for vcstool, see above)
2. add include paths to [.vscode settings](.vscode/c_cpp_properties.json)

### Pull all nested repositories at once
To get remote updates (new tags,..), add new nested repositories, and pull all changes at from remote, run
```bash
vcs custom --args remote update
vcs import src < ros.repos
vcs pull src
```
Find more details in the [official documentation](https://docs.ros.org/en/humble/Installation/Maintaining-a-Source-Checkout.html).

## Build Testing
Always run
```bash
colcon test
```
to see if all tests succeed. Specific information see below
### C++
`uncrustify` is used as c++ code formatting tool, as there is a config file
provided by ROS2. Check your C++ code format with running
```bash
ament_uncrustify src
```
To reformat run
```bash
ament_uncrustify --reformat src
```
### Python
Check your python code format with running
```bash
ament_flake8
```
You can use `autopep8` (`pip install autopep8`) for inplace code formatting with
```bash
autopep8 -riaaa
```

## Setup IDEs
The following IDEs were tested with this workspace.

### Qt Creator
Project folder is included in repository.
#### Tested versions
* 6.0.1 with ROS Qt Plugin 0.5.0

#### Installation instructions
* Follow the instructions for the [ROS Qt Creator Plug-in](https://github.com/ros-industrial/ros_qtc_plugin)

* Additionally, install [beautifier](https://doc.qt.io/qtcreator/creator-beautifier.html) plugin and activate uncrustify. Go to *Tools->Options->Beautifier*
 * *General->Enable auto format on file save* and choose Tool: *uncrustify*
 * Uncrustify: Check *Use file uncrustify.cfg defined in project files*

* For python support, install `Python Language Server`

* To add the project, go to *File -> Open File or Project* and choose `ros2_baustelle_ws.workspace` within this folder.
 Go to your project view: *right click -> Add Existing File* -> choose `src/uncrustify.cfg`

* For debugging you have to change the permissions to be able to attach to a running process. Run in a terminal window
 ```bash
 sudo bash -c "echo 0 > /proc/sys/kernel/yama/ptrace_scope"
 ```
Compile the project and launch it from the terminal. Then go to *Qt Creator -> Debug -> Start Debugging -> Attach to a Running Application* and choose the node/process you want to debug. Then you can set breakpoints, watches etc.

#### Known issues
With Qt Creator 6 and the plug-in 0.5.0 it is not possible to create a [run configuration yet](https://github.com/ros-industrial/ros_qtc_plugin/issues/459) -> use the terminal.

### VSCode
Project folder is included in repository.

#### Installation instructions
If you do not use the dockerfile, install the extensions provided by the [devcontainer.json file](.devcontainer/devcontainer.json). E.g.,install [uncrustify](https://marketplace.visualstudio.com/items?itemName=zachflower.uncrustify) extension. 

