#!/usr/bin/env python3

import rosbag
import math
import argparse
from visualization_msgs.msg import Marker
import os
import re
import csv
from collections import defaultdict

NParticlesLogged = True

bag_folder = "/home/appuser/livox_mav_track/scripts/results/grid_search_stds/"

def compute_distance(x, y, z):
    return math.sqrt(x**2 + y**2 + z**2)

def find_max_distance_and_tracking_percentage(bag_path):
    max_distance = 0.0
    total_valid_msgs = 0
    tracked_msgs = 0

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/mav_track/mav_marker']):
            if msg.ns == "mav_model":
                # Count messages and track if UAV is currently being tracked
                total_valid_msgs += 1
                if msg.color.a == 1.0:
                    tracked_msgs += 1
                    pos = msg.pose.position
                    distance = compute_distance(pos.x, pos.y, pos.z)
                    # that is a false background measurement for sure
                    if distance > 150:
                        continue
                    if distance > max_distance:
                        max_distance = distance

    tracking_percentage = (tracked_msgs / total_valid_msgs * 100) if total_valid_msgs > 0 else 0.0
    return max_distance, tracking_percentage

if __name__ == "__main__":
    
    bag_names = [f for f in os.listdir(bag_folder) if f.endswith('.bag')]
    
    bag_names.sort()
    
    all_experiments = []
    
    for bag_name in bag_names:
        bag_path = f"{bag_folder}{bag_name}"
        if NParticlesLogged: 
            match = re.search(
                r"std_pos-(?P<std_pos>[\d.]+)_std_vel-(?P<std_vel>[\d.]+)_std_acc-(?P<std_acc>[\d.]+)_std_sense-(?P<std_sense>[\d.]+)_NParticles-(?P<NParticles>\d+)",
                bag_name
            )
            if match:     
                NParticles = int(match.group("NParticles"))
            else:
                print(f"Params not found in {bag_name}")
                continue
        else:
            match = re.search(
                r"std_pos-(?P<std_pos>[\d.]+)_std_vel-(?P<std_vel>[\d.]+)_std_acc-(?P<std_acc>[\d.]+)_std_sense-(?P<std_sense>[\d.]+)",
                bag_name
            )
            if match:     
                NParticles = 5000
            else:
                print(f"Params not found in {bag_name}")
                continue          

        if match:
            std_pos = float(match.group("std_pos"))
            std_vel = float(match.group("std_vel"))
            std_acc = float(match.group("std_acc"))
            std_sense = float(match.group("std_sense"))
            
        max_dist, tracking_pct = find_max_distance_and_tracking_percentage(bag_path)
        
        print(f"std_pos={std_pos}, std_vel={std_vel}, std_acc={std_acc}, std_sense={std_sense}, NParticles={NParticles}: max_dist={max_dist:.2f} m, tracking_pct={tracking_pct:.2f}%")
        
        csv_path = os.path.join(bag_folder, "results.csv")
        file_exists = os.path.isfile(csv_path)

        all_experiments.append([
            bag_name,
            std_pos,
            std_vel,
            std_acc,
            std_sense,
            NParticles,
            max_dist,
            tracking_pct
        ])
        
        with open(csv_path, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists:
                writer.writerow(['bag_name', 'std_pos', 'std_vel', 'std_acc', 'std_sense', 'NParticles', 'max_dist', 'tracking_pct'])
            writer.writerow([
                bag_name,
                std_pos,
                std_vel,
                std_acc,
                std_sense,
                NParticles,
                max_dist,
                tracking_pct
            ])
    
    # Group experiments by parameter combination
    grouped = defaultdict(list)
    for exp in all_experiments:
        key = (exp[1], exp[2], exp[3], exp[4], exp[5])  # std_pos, std_vel, std_acc, std_sense
        grouped[key].append(exp)

    summary_csv_path = os.path.join(bag_folder, "summary_results.csv")
    with open(summary_csv_path, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header
        header = [
            'std_pos', 'std_vel', 'std_acc', 'std_sense', 'NParticles',
            'av_max_dist', 'av_tracking_pct'
        ]
        # Add max_dist and tracking_pct columns for each measurement (assuming 5)
        for i in range(1, 6):
            header.append(f'max_dist{i}')
        for i in range(1, 6):
            header.append(f'tracking_pct{i}')
        writer.writerow(header)

        for key, exps in grouped.items():
            std_pos, std_vel, std_acc, std_sense, NParticles = key
            max_dists = [e[6] for e in exps]
            tracking_pcts = [e[7] for e in exps]
            av_max_dist = sum(max_dists) / len(max_dists)
            av_tracking_pct = sum(tracking_pcts) / len(tracking_pcts)
            row = [
                std_pos, std_vel, std_acc, std_sense, NParticles,
                av_max_dist, av_tracking_pct
            ] + max_dists + tracking_pcts
            writer.writerow(row)