import os
import re
import math
import rosbag
from collections import defaultdict
from geometry_msgs.msg import Point
import glob
import csv
from collections import defaultdict
import statistics
import pandas as pd

bag_folder = "/home/appuser/livox_mav_track/scripts/results/max_dist_and_coverage/"
point_numbers_folder = "/home/appuser/livox_mav_track/scripts/results/outside_test/"
GPS_accuracy = 4.0

# from plot_point_numbers_to_map_big_fig.py
max_distances_with_measurement = {
    'simple_distance': 134.69,
    'vertical': 146.86,
    'horizontal': 142.20,
    'horizontal_fast': 141.25,
    'horizontal_fast_fix': 129.76
}

def compute_distance(x, y, z):
    return math.sqrt(x**2 + y**2 + z**2)

def point_from_marker(marker_msg):
    return marker_msg.points[-1] if marker_msg.points else None

def find_closest_gps(gps_msgs, target_time):
    # gps_msgs is list of tuples: (time, Point)
    closest = None
    min_diff = float('inf')
    for t, pt in gps_msgs:
        diff = abs((target_time - t).to_sec())
        if diff < min_diff:
            min_diff = diff
            closest = pt
    return closest

def find_max_distance_and_tracking_stats(bag_path):
    column_names = ['x', 'y', 'z', 'GPS_x', 'GPS_y', 'GPS_z', 'distance', 'time_s', 'time_ns']
    data = pd.DataFrame(columns=column_names)
    
    bag_csv_path = os.path.splitext(bag_path)[0] + ".csv"

    if os.path.exists(bag_csv_path):
        print(f"Loading precomputed results from {bag_csv_path}")
        data = pd.read_csv(bag_csv_path)
    else:
        print(f"No precomputed results found for {bag_path}. Processing the bag file.")
        gps_data = []
        data_list = []

        with rosbag.Bag(bag_path, 'r') as bag:
            # Read all GPS messages into a list first
            for topic, msg, t in bag.read_messages(topics=['/gps_marker']):
                pt = point_from_marker(msg)
                if pt:
                    gps_data.append((t, pt))

            for topic, msg, t in bag.read_messages(topics=['/mav_track/mav_marker']):
                if msg.ns == "mav_model":
                    pos = msg.pose.position
                    # Save the closest GPS data in time
                    closest_pt = find_closest_gps(gps_data, t)
                    if closest_pt:
                        data_list.append({
                            'x': pos.x,
                            'y': pos.y,
                            'z': pos.z,
                            'GPS_x': closest_pt.x,
                            'GPS_y': closest_pt.y,
                            'GPS_z': closest_pt.z,
                            'distance': compute_distance(pos.x, pos.y, pos.z),
                            'time_s': t.secs,
                            'time_ns': t.nsecs
                        })
                    else:
                        data_list.append({
                            'x': pos.x,
                            'y': pos.y,
                            'z': pos.z,
                            'GPS_x': None,
                            'GPS_y': None,
                            'GPS_z': None,
                            'distance': compute_distance(pos.x, pos.y, pos.z),
                            'time_s': t.secs,
                            'time_ns': t.nsecs
                        })
                        
        data = pd.DataFrame(data_list)
        data.to_csv(bag_csv_path, index=False)
        
    all_mav_msgs = len(data)

    max_det_distance = None
    for key, value in max_distances_with_measurement.items():
        if key in bag_path:
            max_det_distance = value
            break
    
    # Iterate over the rows of data to determine if each MAV message is "tracked"
    is_tracked = []
    for idx, row in data.iterrows():
        if pd.notnull(row['GPS_x']) and pd.notnull(row['GPS_y']) and pd.notnull(row['GPS_z']):
            delta = compute_distance(
                row['x'] - row['GPS_x'],
                row['y'] - row['GPS_y'],
                row['z'] - row['GPS_z'],
            )
            dist = row['distance']
            if (delta <= GPS_accuracy) and (dist < max_det_distance):
                is_tracked.append(True)
            else:
                is_tracked.append(False)
        else:
            is_tracked.append(False)

    is_tracked_series = pd.Series(is_tracked, index=data.index)
    max_distance = data.loc[is_tracked_series, 'distance'].max()

    max_idx = data.loc[is_tracked_series, 'distance'].idxmax()
    max_row = data.loc[max_idx]
    max_gps_x = max_row['GPS_x']
    max_gps_y = max_row['GPS_y']
    max_gps_z = max_row['GPS_z']

    max_gps = [ max_gps_x, max_gps_y, max_gps_z ]
    
    tracking_percentage = (sum(is_tracked) / all_mav_msgs) * 100

    return max_distance, tracking_percentage, max_gps

#  test, original_bag, params, run
def parse_filename(filename):
    match = re.match(r'(.*?)_(.*?)_(.*?)run(\d+)\.bag', os.path.basename(filename))
    if match:
        return match.groups()
    else:
        return ('unknown', 'unknown', 'unknown', '0')

def main():
    csv_path = os.path.join(bag_folder, "results.csv")
    summary_csv_path = os.path.join(bag_folder, "summary_results.csv")

    bag_files = sorted(glob.glob(os.path.join(bag_folder, '*.bag')))
    detailed_results = []
    grouped = defaultdict(list)  # Key: (Test, Params), Value: list of result dicts

    if os.path.exists(csv_path):
        print(f"{csv_path} already exists. Loading from there.")
        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            detailed_results = [row for row in reader]
            # Convert numeric fields to float/int as needed
            for row in detailed_results:
                row['Run'] = int(row['Run'])
                row['MaxDist'] = float(row['MaxDist'])
                row['MaxGPS_x'] = float(row['MaxGPS_x'])
                row['MaxGPS_y'] = float(row['MaxGPS_y'])
                row['MaxGPS_z'] = float(row['MaxGPS_z'])
                row['TrackPct'] = float(row['TrackPct'])
                grouped[(row['Test'], row['Original_bag'], row['Params'])].append(row)
        
    else: 
        for bag_file in bag_files:
            test, original_bag, params, run = parse_filename(bag_file)
            run = int(run)
            max_dist, track_pct, max_gps = find_max_distance_and_tracking_stats(bag_file)
            print(f"Processed {bag_file}: MaxDist={max_dist}, TrackPct={track_pct}")
            
            max_return_gps_distance = 0  # default value

            if "simple_distance" in bag_file:
                filtered_csv = os.path.join(bag_folder, f"{point_numbers_folder}simple_distance_result.csv")
            elif "vertical" in bag_file:
                filtered_csv = os.path.join(bag_folder, f"{point_numbers_folder}vertical_distance_result.csv")
            elif "horizontal" in bag_file and not "horizontal_fast" in bag_file:
                filtered_csv = os.path.join(bag_folder, f"{point_numbers_folder}horizontal_distance_result.csv")
            elif "horizontal_fast" in bag_file and not "horizontal_fast_fix" in bag_file:
                filtered_csv = os.path.join(bag_folder, f"{point_numbers_folder}horizontal_fast_distance_result.csv")
            elif "horizontal_fast_fix" in bag_file:
                filtered_csv = os.path.join(bag_folder, f"{point_numbers_folder}horizontal_fast_fix_distance_result.csv")
            else:
                print("problem1")
                return

            if filtered_csv and os.path.exists(filtered_csv):
                df = pd.read_csv(filtered_csv)
                # Find the row with max_gps coordinates
                mask = (
                    (df['gps_x'] == max_gps[0]) &
                    (df['gps_y'] == max_gps[1]) &
                    (df['gps_z'] == max_gps[2])
                )
                idx = df[mask].index
                if not idx.empty:
                    row_idx = idx[0]
                    if df.loc[row_idx, 'count'] != 0:
                        max_return_gps_distance = df.loc[row_idx, 'distance']
                    else:
                        # Search previous rows for nonzero count
                        prev_rows = df.loc[:row_idx-1][df['count'] != 0]
                        if not prev_rows.empty:
                            max_return_gps_distance = prev_rows.iloc[-1]['distance']
            else:
                print("problem2")
                return
            
            result = {
                'Test': test,
                'Original_bag': original_bag,
                'Params': params,
                'Run': run,
                'MaxDist': max_dist,
                'MaxGPS_x': max_gps[0],
                'MaxGPS_y': max_gps[1],
                'MaxGPS_z': max_gps[2],
                'MaxReturnGPSdistance': max_return_gps_distance,
                'TrackPct': track_pct,
            }
            detailed_results.append(result)
            grouped[(test, original_bag, params)].append(result)

        # Write detailed CSV
        with open(csv_path, 'w', newline='') as f:
            fieldnames = ['Test', 'Original_bag', 'Params', 'Run', 'MaxDist', 'MaxGPS_x', 'MaxGPS_y', 'MaxGPS_z', 'MaxReturnGPSdistance', 'TrackPct']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in detailed_results:
                writer.writerow(row)
        print(f"Detailed results saved to: {csv_path}")

    # Build and write summary CSV
    with open(summary_csv_path, 'w', newline='') as f:
        fieldnames = (
            ['Test', 'Original_bag', 'Params',
             'Average_MaxDist', 'Std_MaxDist',
             'Average_MaxReturnGPSdistance', 'Std_MaxReturnGPSdistance',
             'Average_TrackPct', 'Std_TrackPct'] +
            [f'MaxDist{i+1}' for i in range(5)] +
            [f'MaxReturnGPSdistance{i+1}' for i in range(5)] +
            [f'TrackPct{i+1}' for i in range(5)]
        )
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for (test, original_bag, params), runs in grouped.items():
            # Sort runs by their run index to ensure ordering
            runs_sorted = sorted(runs, key=lambda r: r['Run'])
            if len(runs_sorted) != 5:
                print(f"Warning: {test} - {params} has {len(runs_sorted)} runs (expected 5)")

            max_dists = [r['MaxDist'] for r in runs_sorted]
            track_pcts = [r['TrackPct'] for r in runs_sorted]
        
            row = {
                'Test': test,
                'Original_bag': original_bag,
                'Params': params,
                'Average_MaxDist': sum(max_dists) / len(max_dists),
                'Std_MaxDist': statistics.stdev(max_dists) if len(max_dists) > 1 else 0.0,
                'Average_MaxReturnGPSdistance': sum(float(r['MaxReturnGPSdistance']) for r in runs_sorted) / len(runs_sorted),
                'Std_MaxReturnGPSdistance': statistics.stdev(float(r['MaxReturnGPSdistance']) for r in runs_sorted) if len(runs_sorted) > 1 else 0.0,
                'Average_TrackPct': sum(track_pcts) / len(track_pcts),
                'Std_TrackPct': statistics.stdev(track_pcts) if len(track_pcts) > 1 else 0.0,
            }

            for i in range(5):
                row[f'MaxDist{i+1}'] = max_dists[i] if i < len(max_dists) else ''
                row[f'MaxReturnGPSdistance{i+1}'] = runs_sorted[i]['MaxReturnGPSdistance'] if i < len(runs_sorted) else ''
                row[f'TrackPct{i+1}'] = track_pcts[i] if i < len(track_pcts) else ''           

            writer.writerow(row)

    print(f"Summary results saved to: {summary_csv_path}")

if __name__ == "__main__":
    main()


