#!/bin/bash

# Paths
CONFIG_ORIGINAL="/home/appuser/livox_mav_track/src/my_packages/mav_track_ros/mav_track/config/mav_track_inside.yaml"
CONFIG_BACKUP="/home/appuser/livox_mav_track/src/my_packages/mav_track_ros/mav_track/config/mav_track_inside_backup.yaml"
OUTPUT_DIR="/home/appuser/livox_mav_track/scripts/results/accuracy/"
BAG_FILES=("/home/appuser/_Datasets/livox_mav_track/inside/horizontal.bag" 
           "/home/appuser/_Datasets/livox_mav_track/inside/vertical.bag" 
           "/home/appuser/_Datasets/livox_mav_track/inside/fast.bag" 
           "/home/appuser/_Datasets/livox_mav_track/inside/lostandfound.bag")
declare -A BAG_START_TIMES
declare -A BAG_DURATION_TIMES
declare -A RESET_TIMES
BAG_START_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/horizontal.bag"]=5.0
BAG_START_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/vertical.bag"]=5.0
BAG_START_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/fast.bag"]=5.0
BAG_START_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/lostandfound.bag"]=5.0
BAG_DURATION_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/horizontal.bag"]=10000
BAG_DURATION_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/vertical.bag"]=82
BAG_DURATION_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/fast.bag"]=55
BAG_DURATION_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/lostandfound.bag"]=67
RESET_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/horizontal.bag"]=18.0
RESET_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/vertical.bag"]=18.0
RESET_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/fast.bag"]=18.0
RESET_TIMES["/home/appuser/_Datasets/livox_mav_track/inside/lostandfound.bag"]=18.0

# Topics to record
RECORD_TOPICS="/tf /vrpn_client_node/turret/pose /vrpn_client_node/cf1/pose /mav_track/mav_marker /mav_track/particles"

mkdir -p "$OUTPUT_DIR"

# Function to convert bool to lowercase string
bool_to_str() {
    if [ "$1" == true ] || [ "$1" == "true" ]; then
        echo "true"
    else
        echo "false"
    fi
}

cp "$CONFIG_ORIGINAL" "$CONFIG_ORIGINAL"

# Tests defined as an array of associative arrays
declare -A TEST1=( ["Tracking_Type"]=1 ["KF_Motion_Model_Type"]=0 ["KF_Search_Radius"]=0.12 ["KF_Dogru_Use_Min_Search_Radius"]=false )
declare -A TEST2=( ["Tracking_Type"]=1 ["KF_Motion_Model_Type"]=2 ["KF_Search_Radius"]=0.20 )
declare -A TEST3=( ["Tracking_Type"]=0 ["ParticleFilter_Motion_Model_Type"]=1 )
declare -A TEST4=( ["Tracking_Type"]=0 ["ParticleFilter_Motion_Model_Type"]=3 )

# Group tests
TEST_NAMES=("TEST1" "TEST2" "TEST3" "TEST4")

# Repetitions
REPEAT=5

# Main loop
for TEST_NAME in "${TEST_NAMES[@]}"; do
    declare -n TEST_PARAMS="$TEST_NAME"

    for BAG_FILE in "${BAG_FILES[@]}"; do
        BAG_BASE=$(basename "$BAG_FILE" .bag)

        for i in $(seq 1 $REPEAT); do
            echo "Running $TEST_NAME with $BAG_FILE (Run $i)..."

            # Create unique suffix for filename
            PARAM_STR=""

            # Apply yq changes and generate param string
            for KEY in "${!TEST_PARAMS[@]}"; do
                VALUE="${TEST_PARAMS[$KEY]}"

                if [[ "$VALUE" == "true" || "$VALUE" == "false" ]]; then
                    VALUE=$(bool_to_str "$VALUE")
                    yq -i ".${KEY} = ${VALUE}" "$CONFIG_ORIGINAL"
                elif [[ "$VALUE" =~ ^[0-9.]+$ ]]; then
                    yq -i ".${KEY} = ${VALUE}" "$CONFIG_ORIGINAL"
                else
                    yq -i ".${KEY} = ${VALUE}" "$CONFIG_ORIGINAL"
                fi

                PARAM_STR+="${KEY}_${VALUE}_"
            done

            OUTPUT_NAME="${OUTPUT_DIR}/${TEST_NAME}_${BAG_BASE}_${PARAM_STR}run${i}.bag"

            if [ -f "$OUTPUT_NAME" ]; then
              echo "Skipping existing bag: $OUTPUT_NAME"
              continue
            fi
            
            YAML_COPY_NAME="${OUTPUT_DIR}/${TEST_NAME}_${BAG_BASE}_${PARAM_STR}run${i}.yaml"

            cp "$CONFIG_ORIGINAL" "$YAML_COPY_NAME"

            roslaunch mav_track mav_track_inside.launch &
            LAUNCH_PID=$!
            sleep 5  # Give time to initialize

            echo "------------------------------------------------launch---------------------------------------------"

            # Start recording
            rosbag record -O "$OUTPUT_NAME" $RECORD_TOPICS &
            RECORD_PID=$!
            sleep 2  # Small delay before playback

            echo "------------------------------------------------record------------------------------------------------"

            # Start playback
            START_TIME="${BAG_START_TIMES[$BAG_FILE]}"
            DURATION=${BAG_DURATION_TIMES[$BAG_FILE]}
            RESET_TIME=${RESET_TIMES[$BAG_FILE]}
            rosbag play --duration 1 --clock "$BAG_FILE" --topics /tf_static
            rosbag play "$BAG_FILE" --start "$START_TIME" --duration "$DURATION" --clock &
            PLAY_PID=$!

            echo "------------------------------------------------play------------------------------------------------"

            # Check if Tracking_Type is 1
            if [[ "${TEST_PARAMS[Tracking_Type]}" == "1" ]]; then
                echo "Calling KF reset service because Tracking_Type is 1..."              
                sleep $RESET_TIME
                rosservice call /mav_track/reset_kf "{}"
                echo "------------------------------------------------service------------------------------------------------"
            else
                echo "Calling particle filter reset service because Tracking_Type is 0..."
                sleep $RESET_TIME
                rosservice call /mav_track/particle_reset "{}"
                echo "------------------------------------------------service------------------------------------------------"
            fi 
            sleep 2  # Give it a moment to finish

            wait $PLAY_PID

            # Kill recording and launch processes
            kill $RECORD_PID
            kill $LAUNCH_PID

            echo "------------------------------------------------kill------------------------------------------------"
            sleep 3

        done
    done
done

# Clean up
rm "$CONFIG_ORIGINAL"
cp "$CONFIG_BACKUP" "$CONFIG_ORIGINAL"
rm "$CONFIG_BACKUP"


echo "Outside tests completed. Results are in $OUTPUT_DIR"