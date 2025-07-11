# LiVOX_MAV_track

This is the code and data accompanying the paper "Detection and Tracking of MAVs Using a Rosette Scanning Pattern LiDAR".

*Cite info coming soon.*

## Table of Contents
- [Install](#install)
- [Traditional install](#traditional-install)
- [Usage](#usage)
- [Dataset](#dataset)
- [Reproducing results](#reproducing-results)
- [Acknowledgements](#acknowledgements)
- [License](#license)

## Install

We provide a fully containerized environment to run the examples. This contains all dependencies and tools to be able to simply try out the project and reproduce the results in our paper. You will need some LINUX distro (WSL should also work) with docker cli and vscode installed.

```bash
git clone https://github.com/gsanya/LiVOX_MAV_track.git
cd LiVOX_MAV_track/misc/docker
./build_docker.sh
```

After it is built (takes some time), you can run it and attach vscode to it. For that you will need the following containers installed:
- Dev Containers
- Container Tools

Start the container:
```bash
./run_docker.sh
```
*There is a .params file in the docker folder (created during the build process). You should update the dataset_folder to point to the folder where you downloaded the dataset to. The other params do not need changing.*

Attach to it using the Containers Side Bar in the Activity Bar by right clicking on the running container and selecting Attach Visual Studio Code. This will open an instance of vscode "inside the container". You should open the folder `/home/appuser/livox_mav_track` and the outside of the vscode should become yellow. You have the fully installed project with dependencies in the container now. You can go to [Usage](#susage) to try out the algorithm, or go to [Reproducing results](#reproducing-results) to find instructions on how to reproduce our results. Use the terminals inside this vscode instance for everything.

## Traditional install

The project is using ROS Noetic. You can install the dependencies based on `misc/docker/Dockerfile` and than it should be fairly simple to put the `my_packages/mav_track_ros` folder to any catkin workspace and build it with the configs from the `Dockerfile`.

Tracy can cause some build problems. Include it based on the the `Dockerfile` or remove it. In the code, you mainly need to remove the includes and the `ZoneScoped;` lines.

If you need more help, open an issue. 

## Usage

### Outdoor experiments

Main algorithm:
```bash
roslaunch uav_track uav_track_outside.launch
```
Visualization:
```bash
rviz -d /home/appuser/drone_following_LIVOX/src/uav_track_ros/uav_track/config/outside.rviz
```
*You should start a rosbag with `/livox/lidar` topic for the LiDAR point clouds. Also make sure to have a transform between the `livox_frame` and `world`. With the dataset release we will provide further info on how to run it.*

### Indoor experiment

Main algorithm:
```bash
roslaunch uav_track uav_track_inside.launch
```
Visualization:
```bash
rviz -d /home/appuser/drone_following_LIVOX/src/uav_track_ros/uav_track/config/inside.rviz
```

## Dataset

*coming soon*

## Reproducing Results

*scripts coming soon with dataset*

## Acknowledgements

Parts of this code are adapted from the following repositories:

- [dynamic_scan_tracking](https://github.com/TIERS/dynamic_scan_tracking/tree/main): Code for EKF
- [rviz2-panel](https://github.com/BruceChanJianLe/rviz2-panel): rviz2 button panel

We thank the original authors for open-sourcing their work.

## License

This code is released under the MIT License. See `LICENSE` file for details.

Note: The files adapted from [dynamic_scan_tracking] and [rviz2-panel] are also licensed under MIT License but check their `LICENSE` file.