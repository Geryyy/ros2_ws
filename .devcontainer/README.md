# Dockerfile buildup
* use as few layers (i.e., RUN-commands) as possible to reduce disk storage if no multi-stage setup is used, see the [official docker documentation](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers)
* priorize layers from "won't change at all" down to "might change regularily" to decrease build time
if anything changed
* [sort multi-line arguments](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#sort-multi-line-arguments)

# Host Configuration

In order to run the Docker container in this repository, perform the following modifications on the host system.

__warning: the apt keys in the following description might have changed__

## Network Stack
To be able to run the cameras with no dropped frames, but also to gain reasonable ros messaging performance, increase the network stack size by adding a file:

`/etc/sysctl.d/99-network-stack-size.conf`

with the content of:
```
net.core.rmem_max = 20242880
net.core.wmem_max = 20242880
net.core.rmem_default = 20242880
net.core.wmem_default = 20242880
```

and reboot.

## Docker
### Linux
For installing see the [official docker documentation](
https://docs.docker.com/engine/install/ubuntu/#install-docker-engine).

Add current non root user to docker (and _dialout_?) group:
```sh
sudo usermod -aG docker $USER
```
and reboot.

### Windows
Uncomment the last line in the [vscode config file](devcontainer.json) to set the correct environment variables automatically.
To enable GUI application support for the docker container, follow these steps:
- Install [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/)
- Install & start XLaunch (click Next button until you get to 'Extra settings' settings screen). On the 'Extra settings' settings screen, check the checkbox next to the text 'Disable Access Control'. This will disable server access control.
- Check if GUI applications work, e.g. by running `ros2 run rviz2 rviz`

If this doesn't work:
- Get your IP address by typing `ipconfig` in the Windows Command Prompt (search for _Ethernet-Adapter vEthernet (WSL)_) and check in the vscode bash with 
```sh
 echo $DISPLAY
```
if the correct address is returned.
- Check the last line in your ~/.bashrc if the command from the [updateContentCommand script](updateContentCommand.sh) added the correct line. You can manually set the display environment variable in a terminal via
```sh
export DISPLAY={your_ip_address}:0.0
```

As by now, it seems that WLS2 does not support hardware-accelerated rendering for OpenGL>1.4. Install
```sh
 sudo apt install mesa-utils
```
and check the graphics details with
```sh
 glxinfo -B
```
and the performance with (open the performance view of the Windows Task manager to see if GPU is used.)
```sh
 glxgears
```
1. Launching VcxSrv with 'Native opengl' checked, and export
```sh
export LIBGL_ALWAYS_INDIRECT=1
```
This uses OpenGLv1.4 and glxgears runs hardware-accelerated, but rviz2 or gazebo won't run with OpenGLv1.4

2. Launching VcxSrv with 'Native opengl' disabled, and export
```sh
export LIBGL_ALWAYS_INDIRECT=0
```
This uses some MESA driver and software rendering. Everything runs, but slow.


## Nvidia

This is needed only if a Nvidia graphics card is present and is needed by the user. See some uncommented
bunch of code in the Dockerfile.vscode and devcontainer.json
### Nvidia Drivers

Install the latest drivers (that are compatible with the card).

```sh
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt-get update
sudo apt-get -y install cuda-11-1
```

### Nvidia Docker Runtime

```sh
curl https://get.docker.com | sh \
  && sudo systemctl --now enable docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

test with:

```sh
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```
