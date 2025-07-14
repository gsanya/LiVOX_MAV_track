#!/bin/bash

# Define parameter values
param1_values=(0.5)
param2_values=(0.5)
param3_values=(0.0)
param4_values=(0.2)
param5_values=(1000 2000 3000 4000 5000 6000 7000 8000 9000 10000)

# Parameter names in the YAML
param1_name="ParticleFilter_Predict_Std_Pos"
param2_name="ParticleFilter_Predict_Std_Vel"
param3_name="ParticleFilter_Predict_Std_Acc"
param4_name="ParticleFilter_Sensing_Std"
param5_name="ParticleFilter_Nparticles"

# Path to the original and temp config file
CONFIG_ORIGINAL="/home/appuser/livox_mav_track/src/my_packages/mav_track_ros/mav_track/config/mav_track_outside.yaml"
CONFIG_BACKUP="/home/appuser/livox_mav_track/src/my_packages/mav_track_ros/mav_track/config/mav_track_outside_backup.yaml"

# Bag file to play
BAG_FILE="/home/appuser/_Datasets/livox_mav_track/field_test/sunny.bag"

# Topics to record
RECORD_TOPICS="/mav_track/mav_marker"

# Output directory
OUTPUT_DIR="/home/appuser/livox_mav_track/scripts/results/grid_search_NParticles"
mkdir -p "$OUTPUT_DIR"

# Repetitions
REPEAT=1

# Create a temporary YAML file
cp "$CONFIG_ORIGINAL" "$CONFIG_BACKUP"

# Grid search loop
for p1 in "${param1_values[@]}"; do
  for p2 in "${param2_values[@]}"; do
    for p3 in "${param3_values[@]}"; do
      for p4 in "${param4_values[@]}"; do
        for p5 in "${param5_values[@]}"; do          
          for run in $(seq 1 $REPEAT); do
            echo "Running combination: $param1_name=$p1, $param2_name=$p2, $param3_name=$p3, $param4_name=$p4, $param5_name=$p5 (Run $run)"          
            OUTPUT_NAME="${OUTPUT_DIR}/record_std_pos-${p1}_std_vel-${p2}_std_acc-${p3}_std_sense-${p4}_NParticles-${p5}_run-${run}.bag"

            if [ -f "$OUTPUT_NAME" ]; then
              echo "Skipping existing bag: $OUTPUT_NAME"
              continue
            fi

            yq e -i ".$param1_name = $p1" "$CONFIG_ORIGINAL"
            yq e -i ".$param2_name = $p2" "$CONFIG_ORIGINAL"
            yq e -i ".$param3_name = $p3" "$CONFIG_ORIGINAL"
            yq e -i ".$param4_name = $p4" "$CONFIG_ORIGINAL"
            yq e -i ".$param5_name = $p5" "$CONFIG_ORIGINAL"
              
            YAML_COPY_NAME="${OUTPUT_DIR}/params_std_pos-${p1}_std_vel-${p2}_std_acc-${p3}_std_sense-${p4}_NParticles-${p5}_run-${run}.yaml"
            cp "$CONFIG_ORIGINAL" "$YAML_COPY_NAME"
            # Launch the system with the temp config file
            TRACY_PORT=8088 roslaunch mav_track mav_track_outside.launch &
            LAUNCH_PID=$!
            sleep 5  # Give time to initialize

            # start Tracy
            TRACY_SAVE_NAME="${OUTPUT_DIR}/tracy_capture_std_pos-${p1}_std_vel-${p2}_std_acc-${p3}_std_sense-${p4}_NParticles-${p5}_run-${run}.tracy"
            /home/appuser/livox_mav_track/thirdparty/tracy/capture/build/tracy-capture -p 8088 -o ${TRACY_SAVE_NAME} &
            TRACY_PID=$!
            sleep 5  # Give time for Tracy to start
            
            # Start recording
            rosbag record -O "$OUTPUT_NAME" $RECORD_TOPICS &
            RECORD_PID=$!

            sleep 2  # Small delay before playback

            # Start playback
            rosbag play --duration 1 --clock "$BAG_FILE" --topics /tf_static
            rosbag play --start 5.0 --duration 190.0 --clock "$BAG_FILE" 

            sleep 2  # Give it a moment to finish

            # Kill recording and launch processes
            kill $RECORD_PID
            kill $LAUNCH_PID
            # wait for Tracy to finish
            sleep 8
          done
        done
      done
    done
  done
done

# Clean up
rm "$CONFIG_ORIGINAL"
cp "$CONFIG_BACKUP" "$CONFIG_ORIGINAL"
rm "$CONFIG_BACKUP"

echo "Grid search completed. Results are in $OUTPUT_DIR"
