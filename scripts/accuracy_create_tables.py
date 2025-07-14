import rosbag
import math
import numpy as np
import tf2_geometry_msgs
import copy
import matplotlib.pyplot as plt
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Point
from tf.transformations import *
import tf
import os
import pandas as pd
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import glob
import re
from scipy.stats import chi2
import csv
from collections import defaultdict
import re

bag_folder = "/home/appuser/livox_mav_track/scripts/results/accuracy"
start_cut_off_time = 20.0
end_cut_off_time = 10.0

# inverts a TransformStamped object
def invert_TransformStamped(transform):              
    rotation=[transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w]
    translation=[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z]

    transform_matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(translation),
                                                                tf.transformations.quaternion_matrix(rotation))
    inversed_transform_matrix = tf.transformations.inverse_matrix(transform_matrix)
    
    ret=TransformStamped()
    ret.header=copy.deepcopy(transform.header)
    ret.child_frame_id=copy.deepcopy(transform.header.frame_id)
    ret.header.frame_id=copy.deepcopy(transform.child_frame_id)
    ret.transform.translation.x = tf.transformations.translation_from_matrix(inversed_transform_matrix)[0]
    ret.transform.translation.y = tf.transformations.translation_from_matrix(inversed_transform_matrix)[1]
    ret.transform.translation.z = tf.transformations.translation_from_matrix(inversed_transform_matrix)[2]
    ret.transform.rotation.x = tf.transformations.quaternion_from_matrix(inversed_transform_matrix)[0]
    ret.transform.rotation.y = tf.transformations.quaternion_from_matrix(inversed_transform_matrix)[1]
    ret.transform.rotation.z = tf.transformations.quaternion_from_matrix(inversed_transform_matrix)[2]
    ret.transform.rotation.w = tf.transformations.quaternion_from_matrix(inversed_transform_matrix)[3]
    
    return ret
    
# calculates the relative transform - A*B^(-1)
def inverse_times_TransformStamped(transformA, transformB):              
    rotationA=[transformA.transform.rotation.x,transformA.transform.rotation.y,transformA.transform.rotation.z,transformA.transform.rotation.w]
    translationA=[transformA.transform.translation.x,transformA.transform.translation.y,transformA.transform.translation.z]

    rotationB=[transformB.transform.rotation.x,transformB.transform.rotation.y,transformB.transform.rotation.z,transformB.transform.rotation.w]
    translationB=[transformB.transform.translation.x,transformB.transform.translation.y,transformB.transform.translation.z]

    transform_matrixA = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(translationA),
                                                                tf.transformations.quaternion_matrix(rotationA))
    transform_matrixB = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(translationB),
                                                                tf.transformations.quaternion_matrix(rotationB))
    
    inversed_transform_matrixB = tf.transformations.inverse_matrix(transform_matrixB)
    
    output_transform = transform_matrixA.dot(inversed_transform_matrixB)
    
    transform = TransformStamped()
        
    transform.transform.translation.x = tf.transformations.translation_from_matrix(output_transform)[0]
    transform.transform.translation.y = tf.transformations.translation_from_matrix(output_transform)[1]
    transform.transform.translation.z = tf.transformations.translation_from_matrix(output_transform)[2]
    transform.transform.rotation.x = tf.transformations.quaternion_from_matrix(output_transform)[0]
    transform.transform.rotation.y = tf.transformations.quaternion_from_matrix(output_transform)[1]
    transform.transform.rotation.z = tf.transformations.quaternion_from_matrix(output_transform)[2]
    transform.transform.rotation.w = tf.transformations.quaternion_from_matrix(output_transform)[3]
    
    transform.header.frame_id = copy.deepcopy(transformA.child_frame_id)
    transform.child_frame_id = copy.deepcopy(transformB.child_frame_id)
    
    return transform

# calculates the combination (product) of transform A and B --> A*B (B is the first transform that will be applied)
def times_TransformStamped(transformA, transformB):
    rotationA=[transformA.transform.rotation.x,transformA.transform.rotation.y,transformA.transform.rotation.z,transformA.transform.rotation.w]
    translationA=[transformA.transform.translation.x,transformA.transform.translation.y,transformA.transform.translation.z]

    rotationB=[transformB.transform.rotation.x,transformB.transform.rotation.y,transformB.transform.rotation.z,transformB.transform.rotation.w]
    translationB=[transformB.transform.translation.x,transformB.transform.translation.y,transformB.transform.translation.z]

    transform_matrixA = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(translationA),
                                                                tf.transformations.quaternion_matrix(rotationA))
    transform_matrixB = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(translationB),
                                                                tf.transformations.quaternion_matrix(rotationB))
    
    output_transform = transform_matrixA.dot(transform_matrixB)
    
    transform = TransformStamped()
        
    transform.transform.translation.x = tf.transformations.translation_from_matrix(output_transform)[0]
    transform.transform.translation.y = tf.transformations.translation_from_matrix(output_transform)[1]
    transform.transform.translation.z = tf.transformations.translation_from_matrix(output_transform)[2]
    transform.transform.rotation.x = tf.transformations.quaternion_from_matrix(output_transform)[0]
    transform.transform.rotation.y = tf.transformations.quaternion_from_matrix(output_transform)[1]
    transform.transform.rotation.z = tf.transformations.quaternion_from_matrix(output_transform)[2]
    transform.transform.rotation.w = tf.transformations.quaternion_from_matrix(output_transform)[3]
    
    transform.header.frame_id = copy.deepcopy(transformB.header.frame_id)
    transform.child_frame_id = copy.deepcopy(transformA.child_frame_id)
    
    return transform

# transform the position part of pose with transform (multipy from left)
def do_transform_position(pose, transform): 
    position=[pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,1.0]

    transform_rotation=[transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w]
    transform_translation=[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z]

    transform_matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(transform_translation),
                                                                tf.transformations.quaternion_matrix(transform_rotation)) 
    new_position = transform_matrix.dot(position)
    
    new_pose = PoseStamped()

    new_pose.pose.orientation.x=0.0
    new_pose.pose.orientation.y=0.0
    new_pose.pose.orientation.z=0.0
    new_pose.pose.orientation.w=1.0
    new_pose.pose.position.x=new_position[0]
    new_pose.pose.position.y=new_position[1]
    new_pose.pose.position.z=new_position[2]
    
    new_pose.header.frame_id = copy.deepcopy(transform.header.frame_id)
    
    return new_pose

# to be able to use my old code
def marker_to_posestamped(marker):
    if not marker.points:
        rospy.logwarn("Marker has no points.")
        return None

    pose_stamped = PoseStamped()
    pose_stamped.header = marker.header

    # Use the first point for position
    first_point = marker.points[0]
    pose_stamped.pose.position = first_point

    # Identity quaternion for orientation
    pose_stamped.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    return pose_stamped

# creates a csv file from the bag file (no time cut here)
def create_csv_from_bag(bag_path,csv_path):
    
    tracking_type = 0
    if "Tracking_Type_1" in bag_path:
        tracking_type = 1
    
    # particle_avstd will be -1 for EKF-s
    column_names = ['Filter_x', 'Filter_y', 'Filter_z', 'GT_speed', 'error_x', 'error_y', 'error_z', 'distance_error', 'particles_avstd', 'is_tracked', 'time_s', 'time_ns']
    data = pd.DataFrame(columns=column_names)
    
    particles_avstd = 0.1
    last_GT_speed = 0.0
    is_tracked = False
    
    bag = rosbag.Bag(bag_path)
    
    # Transforms to build the transform chain
    turret_pan_tf=TransformStamped()
    turret_tilt_tf=TransformStamped()
    
    #Transforms for static transforms
    #(0.0,0.0,0.55),(0.0,0.0,0.0,1.0)
    word_turret_base_pose=Transform()
    word_turret_base_pose.translation.z=0.55
    word_turret_base_pose.rotation.w=1.0
    
    #(0.0,0.0,0.036),(0.0,0.0,0.0,1.0)
    turret_tilt_surface_pose=Transform()
    turret_tilt_surface_pose.translation.z=0.036
    turret_tilt_surface_pose.rotation.w=1.0
    
    #(0.025,0.0,0.05),(0.0,0.0,0.0,1.0)
    turret_surface_frame_pose=Transform()
    turret_surface_frame_pose.translation.x=0.025
    turret_surface_frame_pose.translation.z=0.05
    turret_surface_frame_pose.rotation.w=1.0
    
    #vector pointing from the Livox record pose to the Livox MIMO pose
    relative_pose_of_livox=TransformStamped()
    turret_transform=TransformStamped()
    livox_chained_transform=TransformStamped()
    rotate_x_90_tf=TransformStamped()
    world_turret_base_tf=TransformStamped()
    turret_tilt_surface_tf=TransformStamped()
    turret_surface_frame_tf=TransformStamped()
    livox_chained_transform_test=TransformStamped()
    
    #MIMO drone pose
    MIMO_drone_pose=PoseStamped()
    last_MIMO_drone_pose=PoseStamped()
    
    #drone pose from particle filter shifted with relative pose
    drone_pose=Point()
    
    #differences        
    data_list=[]
    
    
    #go through the bag file
    for topic, msg, t in bag.read_messages():
        #transforms (for livox frame from static transforms and turret position)
        if topic=="/tf":
            for transform in msg.transforms:
                #this contains the transform between turret base an turret pan
                if transform.header.frame_id=="turret/base_link":
                    turret_pan_tf.header=copy.deepcopy(transform.header)
                    turret_pan_tf.transform=copy.deepcopy(transform.transform)
                    # turret_pan_tf.child_frame_id=copy.deepcopy(transform.child_frame_id)
                    turret_pan_tf.child_frame_id="turret/pan_link_test"
                    turret_pan_tf.header.frame_id="turret/base_link_test"
                #this contains the transform between turret pan an turret tilt               
                if transform.header.frame_id=="turret/pan_link":
                    turret_tilt_tf.header=copy.deepcopy(transform.header)
                    turret_tilt_tf.transform=copy.deepcopy(transform.transform)
                    # turret_tilt_tf.child_frame_id=copy.deepcopy(transform.child_frame_id)
                    turret_tilt_tf.child_frame_id="turret/tilt_link_test"
                    turret_tilt_tf.header.frame_id="turret/pan_link_test"
                    #static trasforms
                    world_turret_base_tf=TransformStamped(copy.deepcopy(transform.header),'turret/base_link_test',word_turret_base_pose)
                    world_turret_base_tf.header.frame_id='world'
                    turret_tilt_surface_tf=TransformStamped(copy.deepcopy(transform.header),'turret/surface_link_test',turret_tilt_surface_pose)
                    turret_tilt_surface_tf.header.frame_id='turret/tilt_link_test'
                    turret_surface_frame_tf=TransformStamped(copy.deepcopy(transform.header),'livox_frame_test',turret_surface_frame_pose)
                    turret_surface_frame_tf.header.frame_id='turret/surface_link_test'

                    livox_chained_transform=TransformStamped()
                    livox_chained_transform=times_TransformStamped(livox_chained_transform,world_turret_base_tf)
                    livox_chained_transform=times_TransformStamped(livox_chained_transform,turret_pan_tf)
                    livox_chained_transform=times_TransformStamped(livox_chained_transform,turret_tilt_tf)
                    livox_chained_transform=times_TransformStamped(livox_chained_transform,turret_tilt_surface_tf)
                    livox_chained_transform=times_TransformStamped(livox_chained_transform,turret_surface_frame_tf)
                    livox_chained_transform.header.frame_id="world"
                    livox_chained_transform.child_frame_id="livox_frame_chained"
            
        # livox frame from MIMO 
        elif topic=="/vrpn_client_node/turret/pose":
            #only if we already have somethign usable in livox_frame_chained
            if livox_chained_transform!=TransformStamped():                
                # There is some conversion between the MIMO coordinates and the VRPN coordinates which results in a rotated orientation of the livox_turret frame
                # This can be seen on the difference between the world coordinates of Motive (x-z is the ground plane) and the world coordinates of ROS (x-y is the ground plane)
                # The coordinate mapping from Motive to ROS is:
                # 1  0  0
                # 0  0 -1
                # 0  1  0
                # which is a +90 rotation around the x axis
                # with quaternions
                rotate_x_90=Transform()
                rotate_x_90.rotation.x=0.707
                rotate_x_90.rotation.w=0.707
                rotate_x_90_tf=TransformStamped(copy.deepcopy(transform.header),'Mimo_world',rotate_x_90)
                rotate_x_90_tf.header.frame_id='world'
                            
                turret_transform = TransformStamped()
                turret_transform.header = copy.deepcopy(msg.header)
                turret_transform.header.frame_id = 'Mimo_world' 
                turret_transform.child_frame_id = 'livox_turret'
                turret_transform.transform.translation.x = msg.pose.position.x
                turret_transform.transform.translation.y = msg.pose.position.y
                turret_transform.transform.translation.z = msg.pose.position.z
                turret_transform.transform.rotation.x = msg.pose.orientation.x
                turret_transform.transform.rotation.y = msg.pose.orientation.y
                turret_transform.transform.rotation.z = msg.pose.orientation.z
                turret_transform.transform.rotation.w = msg.pose.orientation.w
                
                # rotate with the 90 rotation around x transform
                turret_transform = times_TransformStamped(turret_transform,rotate_x_90_tf)
                
                # calculate transform from livox_turret to livox_frame: turret_transform*livox_chained_transform^-1
                relative_pose_of_livox = inverse_times_TransformStamped(turret_transform,livox_chained_transform)
                relative_pose_of_livox.header.frame_id="world"
                relative_pose_of_livox.child_frame_id="new_world_pose"
                                    
                # print(relative_pose_of_livox.transform.translation)               
                
        #drone pose from MIMO
        elif topic=="/vrpn_client_node/cf1/pose":
            last_MIMO_drone_pose=copy.deepcopy(MIMO_drone_pose)
            MIMO_drone_pose.pose.position.x=msg.pose.position.x
            MIMO_drone_pose.pose.position.y=msg.pose.position.y
            MIMO_drone_pose.pose.position.z=msg.pose.position.z
            MIMO_drone_pose.header.stamp=copy.deepcopy(msg.header.stamp)           

        #drone pose from particle filter
        elif topic=="/mav_track/mav_marker":
            if msg.ns=="mav_model":
                if msg.color.a==1.0:
                    is_tracked=True
                continue
            
            if msg.header.stamp==rospy.Time(0):
                msg.header.stamp = t
            
            
            # convert marker to posestamped
            msg=marker_to_posestamped(msg)
                            
            if relative_pose_of_livox!=TransformStamped() and MIMO_drone_pose!=PoseStamped():
                                    
                drone_pose=do_transform_position(msg,relative_pose_of_livox)
                
                position_difference=PoseStamped()
                position_difference.header.stamp=msg.header.stamp
                position_difference.pose.position.x=drone_pose.pose.position.x-MIMO_drone_pose.pose.position.x
                position_difference.pose.position.y=drone_pose.pose.position.y-MIMO_drone_pose.pose.position.y
                position_difference.pose.position.z=drone_pose.pose.position.z-MIMO_drone_pose.pose.position.z
                
                distance_error = math.sqrt(position_difference.pose.position.x**2+position_difference.pose.position.y**2+position_difference.pose.position.z**2)
                if last_MIMO_drone_pose!=PoseStamped():
                    delta_distance = math.sqrt((MIMO_drone_pose.pose.position.x-last_MIMO_drone_pose.pose.position.x)**2 + 
                                            (MIMO_drone_pose.pose.position.y-last_MIMO_drone_pose.pose.position.y)**2 + 
                                            (MIMO_drone_pose.pose.position.z-last_MIMO_drone_pose.pose.position.z)**2)
                    delta_time_s = MIMO_drone_pose.header.stamp.secs - last_MIMO_drone_pose.header.stamp.secs
                    delta_time_ns = MIMO_drone_pose.header.stamp.nsecs - last_MIMO_drone_pose.header.stamp.nsecs
                    full_delta_time = delta_time_s + delta_time_ns / 1000000000.0
                    GT_speed = delta_distance / full_delta_time
                    #if the timestamp difference is too small the speed will be bad
                    if full_delta_time<0.005:
                        GT_speed=last_GT_speed
                    last_GT_speed=copy.deepcopy(GT_speed)
                else:
                    GT_speed=0.0
                
                if tracking_type==1:
                    particles_avstd = -1.0
                    
                new_data = [drone_pose.pose.position.x,
                            drone_pose.pose.position.y,
                            drone_pose.pose.position.z,
                            GT_speed,
                            position_difference.pose.position.x,
                            position_difference.pose.position.y,
                            position_difference.pose.position.z,
                            distance_error,
                            particles_avstd,
                            is_tracked,
                            position_difference.header.stamp.secs,
                            position_difference.header.stamp.nsecs]
                data_list.append(pd.Series(new_data, index=data.columns))
        
        elif topic=="/mav_track/particles":
            x_coordinates = [point.x for point in msg.points]
            y_coordinates = [point.y for point in msg.points]
            z_coordinates = [point.z for point in msg.points]
            
            std_dev_x = np.std(x_coordinates) 
            std_dev_y = np.std(y_coordinates) 
            std_dev_z = np.std(z_coordinates) 
            particles_avstd = (std_dev_x+std_dev_y+std_dev_z)/3
                    
    bag.close()
   
    data = pd.DataFrame(data_list)
    data.to_csv(csv_path, index=False)
    # print(data)
    return

def calculate_rms_from_csvs_with_cutoff(csv_paths, start_cut_off, end_cut_off, confidence_level=0.95, n_bootstrap=1000):
    if isinstance(csv_paths, str):
        csv_paths = [csv_paths]
    
    data_list = []
    for csv_path in csv_paths:
        print(f"Reading {csv_path}")
        data = pd.read_csv(csv_path)
        if data.empty:
            print(f"Warning: {csv_path} is empty. Skipping.")
            continue
        last_time_s = data["time_s"].iloc[-1]
        first_time_s = data["time_s"][0]
        maxtime = data["time_s"].max()
        maxtime = maxtime - first_time_s
        maxtime = maxtime - end_cut_off
        # TODO: this doesn't seem to work
        data = data[((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000 < maxtime]
        data = data[((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000 > start_cut_off]
        data = data.reset_index(drop=True)
        
        data_list.append(data)

    if not data_list:
        print("No valid data across files.")
        return None
    
    combined_data = pd.concat(data_list, ignore_index=True)
           
    
    stdev=np.std(combined_data['distance_error'])
    mean=np.mean(combined_data['distance_error'])
    
    squared_errors = combined_data["distance_error"]**2
    rmse = math.sqrt(np.mean(squared_errors))

    n = len(combined_data)
    
    # Chi-squared CI for RMSE
    alpha = 1 - confidence_level
    lower_chi2 = chi2.ppf(1 - alpha / 2, df=n)
    upper_chi2 = chi2.ppf(alpha / 2, df=n)
    rmse_ci_chi2_lower = np.sqrt(n / lower_chi2) * rmse
    rmse_ci_chi2_upper = np.sqrt(n / upper_chi2) * rmse
    
    # Bootstrapped CI for RMSE
    bootstrap_rmses = []
    for _ in range(n_bootstrap):
        sample = np.random.choice(squared_errors, size=n, replace=True)
        bootstrap_rmses.append(math.sqrt(np.mean(sample)))

    lower_percentile = 100 * (alpha / 2)
    upper_percentile = 100 * (1 - alpha / 2)
    rmse_ci_boot_lower = np.percentile(bootstrap_rmses, lower_percentile)
    rmse_ci_boot_upper = np.percentile(bootstrap_rmses, upper_percentile)

    return mean, stdev, rmse, rmse_ci_chi2_lower, rmse_ci_chi2_upper, rmse_ci_boot_lower, rmse_ci_boot_upper
    
def parse_filename(filename):
    # Example: testX_dataY_paramsZrun3.bag
    match = re.match(r'(.*?)_(.*?)_(.*?)run(\d+)\.bag', os.path.basename(filename))
    if match:
        return match.groups()  # (TEST_NAME, BAG_BASE, PARAM_STR, run index)
    else:
        return ('unknown', 'unknown', 'unknown', '0')

def main():
    bag_files = sorted(glob.glob(os.path.join(bag_folder, '*.bag')))
    
    summary_csv_path = os.path.join(bag_folder, "summary_results.csv")    
    
    with open(summary_csv_path, "w") as summary_file:
        fieldnames = (
            ['Test', 'Original_bag', 'Params', 'Run',
            'Mean', 'StDev', 'RMSE',
            'RMSE_CI_Chi2_Lower', 'RMSE_CI_Chi2_Upper',
            'RMSE_CI_Boot_Lower', 'RMSE_CI_Boot_Upper']
        )
        writer = csv.DictWriter(summary_file, fieldnames=fieldnames)
        writer.writeheader()
        
        for bag_file in bag_files:
            test, original_bag, params, run = parse_filename(bag_file)
            run = int(run)
            csv_path = os.path.splitext(bag_file)[0] + ".csv"
            if not os.path.exists(csv_path):
                print(f"Creating CSV for {bag_file}")
                create_csv_from_bag(bag_file, csv_path)
            mean, stdev, rmse, rmse_ci_chi2_lower, rmse_ci_chi2_upper, rmse_ci_boot_lower, rmse_ci_boot_upper = calculate_rms_from_csvs_with_cutoff(csv_path, start_cut_off_time, end_cut_off_time)
            row = {
                'Test': test,
                'Original_bag': original_bag,
                'Params': params,
                'Run': run,
                'Mean': mean,
                'StDev': stdev,
                'RMSE': rmse,
                'RMSE_CI_Chi2_Lower': rmse_ci_chi2_lower,
                'RMSE_CI_Chi2_Upper': rmse_ci_chi2_upper,
                'RMSE_CI_Boot_Lower': rmse_ci_boot_lower,
                'RMSE_CI_Boot_Upper': rmse_ci_boot_upper,
            }
            writer.writerow(row)
    
    print(f"Summary results saved to: {summary_csv_path}")
           
    summary_squashed_csv_path = os.path.join(bag_folder, "summary_squashed_results.csv")
    
    csv_files = sorted(glob.glob(os.path.join(bag_folder, '*.csv')))
    # remove summary_results.csv
    csv_files = [f for f in csv_files if not f.endswith("summary_results.csv")]
    
    pattern = re.compile(r"(?P<test>[^_]+)_(?P<bagbase>[^_]+)_(?P<paramstr>.+?)run\d+\.csv")
    
    with open(summary_squashed_csv_path, "w") as squashed_file:
        fieldnames = (
            ['Test', 'Original_bag', 'Params',
            'Mean', 'StDev', 'RMSE',
            'RMSE_CI_Chi2_Lower', 'RMSE_CI_Chi2_Upper',
            'RMSE_CI_Boot_Lower', 'RMSE_CI_Boot_Upper' ]
        )
        writer = csv.DictWriter(squashed_file, fieldnames=fieldnames)
        writer.writeheader()
        
        groups = defaultdict(list)
        
        for path in csv_files:
            filename = os.path.basename(path)
            match = pattern.match(filename)
            if match:
                key = (match.group("test"), match.group("bagbase"), match.group("paramstr"))
                groups[key].append(path)                
        for key, file_group in groups.items():
            mean, stdev, rmse, rmse_ci_chi2_lower, rmse_ci_chi2_upper, rmse_ci_boot_lower, rmse_ci_boot_upper = calculate_rms_from_csvs_with_cutoff(file_group, start_cut_off_time, end_cut_off_time)
            row = {
                'Test': key[0],
                'Original_bag': key[1],
                'Params': key[2],
                'Mean': mean,
                'StDev': stdev,
                'RMSE': rmse,
                'RMSE_CI_Chi2_Lower': rmse_ci_chi2_lower,
                'RMSE_CI_Chi2_Upper': rmse_ci_chi2_upper,
                'RMSE_CI_Boot_Lower': rmse_ci_boot_lower,
                'RMSE_CI_Boot_Upper': rmse_ci_boot_upper,
            }
            writer.writerow(row)
    
    print(f"Squashed summary results saved to: {summary_squashed_csv_path}")
            
if __name__ == '__main__':
    main()