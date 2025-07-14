# LiVOX_MAV_track

This is the code and data accompanying the paper "Detection and Tracking of MAVs Using a Rosette Scanning Pattern LiDAR".

*Cite info coming soon.*

A Video demonstration can be found here: [https://youtu.be/ghfjqAnDuag](https://youtu.be/ghfjqAnDuag)

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

After it is built (takes some time), you can run it and attach vscode to it. For that you will need the following extensions installed on the host:
- Dev Containers
- Container Tools

Start the container:
```bash
./run_docker.sh
```
*There is a .params file in the docker folder (created during the build process). You should update the dataset_folder to point to the folder where you downloaded the dataset to. The other params do not need changing.*

Attach to it using the Containers Side Bar in the Activity Bar by right clicking on the running container and selecting Attach Visual Studio Code. This will open an instance of vscode "inside the container". You should open the folder `/home/appuser/livox_mav_track` and the outside of the vscode should become yellow. You have the fully installed project with dependencies in the container now. You can go to [Usage](#usage) to try out the algorithm, or go to [Reproducing results](#reproducing-results) to find instructions on how to reproduce our results. Use the terminals inside this vscode instance for everything.

## Traditional install

The project is using ROS Noetic. You can install the dependencies based on `misc/docker/Dockerfile` and then it should be fairly simple to put the `my_packages/mav_track_ros` folder to any catkin workspace and build it with the configs from the `Dockerfile`.

Tracy can cause some build problems. Include it based on the the `Dockerfile` or remove it. In the code, you mainly need to remove the includes and the `ZoneScoped;` lines.

If you need more help, open an issue. 

## Usage

### Outdoor experiments

These commands can be used to run the tracking algorithm on the outside scenarios except the re-detection and foggy experiments.Specifically on the following bags:
- `field_test/sunny.bag`
- `outside/simple_distance.bag`
- `outside/vertical.bag`
- `outside/horizontal.bag`
- `outside/horizontal_fast.bag`
- `outside/horizontal_fast_fix.bag`

Main algorithm:
```bash
roslaunch mav_track mav_track_outside.launch
```
Visualization:
```bash
rviz -d /home/appuser/livox_mav_track/src/my_packages/mav_track_ros/mav_track/config/outside.rviz
```
Bag playback:
```bash
rosbag play --clock /home/appuser/_Datasets/livox_mav_track/specific_bag
```

*If you run it on your own bag, it should have `/livox/lidar` topic for the LiDAR point clouds. Also make sure to have a transform between the `/livox_frame` and `/world`.*

*The livox ros package is installed inside the container to make testing on live data easier.*

#### Special outdoor experiments

These two experiments were run at different locations and in somewhat special circumstances, resulting in some background parameters being different. For ease of use, we provide dedicated launch files and configs for these scenarios.

Main algorithm:
```bash
roslaunch mav_track mav_track_outside_behind_hedges.launch
roslaunch mav_track mav_track_outside_fog.launch
```
Visualization:
```bash
rviz -d /home/appuser/livox_mav_track/src/my_packages/mav_track_ros/mav_track/config/outside.rviz
```
Bag playback:
```bash
rosbag play --clock /home/appuser/_Datasets/livox_mav_track/field_test/occlusion_behind_hedges.bag
rosbag play --clock /home/appuser/_Datasets/livox_mav_track/field_test/foggy.bag
```
### Indoor experiment

These commands can be used to run the tracking algorithm on the inside scenarios. Specifically on the following bags:
- `inside/horizontal.bag`
- `inside/vertical.bag`
- `inside/fast.bag`
- `inside/lost_and_found.bag`

Main algorithm:
```bash
roslaunch mav_track mav_track_inside.launch
```
Visualization:
```bash
rviz -d /home/appuser/livox_mav_track/src/my_packages/mav_track_ros/mav_track/config/inside.rviz
```
Bag playback:
```bash
rosbag play --clock /home/appuser/_Datasets/livox_mav_track/specific_bag
```

## Dataset

The dataset can be downloaded from here:
[https://drive.google.com/drive/folders/1oAbMKawoX46SkeOWBx-tnwVZjE9vodPv?usp=sharing](https://drive.google.com/drive/folders/1oAbMKawoX46SkeOWBx-tnwVZjE9vodPv?usp=sharing)

*We removed all unnecessary topics such as camera, to reduce the file sizes and not to accidentally include human subjects.*

## Reproducing Results

### Motion Model Parameter Selection

#### Plotting FIGURE 2. and 4.

Run the outside experiments and record:
```bash
mkdir -p /home/appuser/livox_mav_track/scripts/outside_test
rosbag record -O /home/appuser/livox_mav_track/scripts/results/outside_test/simple_distance_result.bag /gps_marker /mav_track/filtered_cloud
rosbag record -O /home/appuser/livox_mav_track/scripts/results/outside_test/vertical_result.bag /gps_marker /mav_track/filtered_cloud
rosbag record -O /home/appuser/livox_mav_track/scripts/results/outside_test/horizontal_result.bag /gps_marker /mav_track/filtered_cloud
rosbag record -O /home/appuser/livox_mav_track/scripts/results/outside_test/horizontal_fast_result.bag /gps_marker /mav_track/filtered_cloud
rosbag record -O /home/appuser/livox_mav_track/scripts/results/outside_test/horizontal_fast_fix_result.bag /gps_marker /mav_track/filtered_cloud
```

Run the script:
```bash
python3 plot_point_numbers_to_map_big_fig.py
```

#### Selection of N:

Start roscore in a terminal.

You can inspect the tests in rviz while they run:
```bash
rviz -d /home/appuser/livox_mav_track/src/my_packages/mav_track_ros/mav_track/config/outside.rviz
```

Run the script:
```bash
cd /home/appuser/livox_mav_track/scripts
./param_optim_in_grid_Nparticles.sh
```
This will run 10 tests and save the results (bag file with drone locations, config file, tracy trace). Then the traces can be examined using the profiler:
```bash
/home/appuser/livox_mav_track/thirdparty/tracy/profiler/build/tracy-profiler
```

#### Running the grid search:

You should start roscore and you can also visualize with rviz as before.

```bash
cd /home/appuser/livox_mav_track/scripts
./param_optim_in_grid.sh
```
This will run 5\*3\*3\*5\*3=675 tests, each taking ~250 seconds, and saves the results (bag file with drone locations, config file).

Running the following script creates tables with max distance and tracking coverage:
```bash
python3 grid_search_stds_create_tables.py
```
Finally, the following script creates FIGURE 3. from the tables:
```bash
python3 plot_param_grid.py
```

### Ablation study

The table in this subsection is created using the scripts for 'Tracking performance comparison' by changing the bag names and configs.

### Tracking performance comparison

You should start roscore and you can also visualize with rviz as before.

To run the tests:
```bash
cd /home/appuser/livox_mav_track/scripts
./run_outside_tests.sh
```

This will run 4\*5\*5=100 tests, each ranging from  90-250 seconds, and it saves the results (bag file with drone locations and GPS location, config file).

Run the script to get the data for TABLE 3.:
```bash
python3 max_dist_and_coverage_create_tables.py
```

### Field test

Run the sunny or foggy experiments and record:
```bash
mkdir -p /home/appuser/livox_mav_track/scripts/results/field_test
rosbag record -O /home/appuser/livox_mav_track/scripts/results/field_test/sunny_result.bag /mav_track/mav_marker /mav_track/particles /mav_track/filtered_cloud
rosbag record -O /home/appuser/livox_mav_track/scripts/results/field_test/foggy_result.bag /mav_track/mav_marker /mav_track/particles /mav_track/filtered_cloud
```

Plot with the script (comment out either the sunny or the foggy configs, and play around with start time based on the first detection):
```bash
python3 plot_3D.py
```

### Accuracy

You should start roscore and you can also visualize with rviz as before.

To run the tests:
```bash
cd /home/appuser/livox_mav_track/scripts
./run_inside_tests.sh
```
This will run 4\*4\*5=80 tests, each ranging from 80-100 seconds, and it saves the results (bag file with drone locations and GPS location, config file).

Run the script to get the data for TABLE 4.:
```bash
python3 accuracy_create_tables.py
```

To plot FIGURE 8.:
```bash
python3 plot_finding_the_drone.py
```

Use the following script to get the plots in FIGURE 10.:
```bash
python3 plot_accuracy.py
```
*Change the input bag and max and min time in the above scripts as needed.*

### Losing and regaining tracking

Use the following script to get the plots in FIGURE 12.:
```bash
python3 plot_accuracy.py
```
*Change the input bags and max and min time as needed.*

## Acknowledgements

Parts of this code are adapted from the following repositories:

- [dynamic_scan_tracking](https://github.com/TIERS/dynamic_scan_tracking/tree/main): Code for EKF
- [rviz2-panel](https://github.com/BruceChanJianLe/rviz2-panel): rviz2 button panel

We thank the original authors for open-sourcing their work.

## License

This code is released under the MIT License. See `LICENSE` file for details.

Note: The files adapted from [dynamic_scan_tracking] and [rviz2-panel] are also licensed under MIT License but check their `LICENSE` file.