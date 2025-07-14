import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.gridspec as gridspec
from matplotlib.colors import ListedColormap
import os
import pandas as pd

textsize=15
circle_size=15
colorbar_spacing = 0.1

vert_color_bar = False
gps_accuracy = 4.0
bag_folder = "/home/appuser/livox_mav_track/scripts/results/outside_test"

output_figure_name = "/home/appuser/livox_mav_track/scripts/figs/combined_overview.png"
bag_files = [
    "simple_distance_result.bag",
    "vertical_result.bag",
    "horizontal_result.bag",
    "horizontal_fast_result.bag",
    "horizontal_fast_fix_result.bag"
]

fig_names = [
    "Simple distance",
    "Vertical",
    "Horizontal slow",
    "Horizontal fast",
    "Horizontal fast fix"
]

# [start_cut_sec, end_cut_sec]
bag_time_limits = [
    [0, 10],
    [0, 9],
    [10, 0],
    [10, 15],
    [10, 15],
]

# Axes to visualize per bag
bag_axes = [
    ['x', 'y'],
    ['x', 'z'],
    ['x', 'y'],
    ['x', 'y'],
    ['x', 'y'],
]

def point_from_marker(marker_msg):
    return marker_msg.points[-1] if marker_msg.points else None

def pointcloud2_to_xyz_array(cloud_msg):
    points = []
    for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])
    return np.array(points)

def process_bag(bag_path, gps_accuracy, time_limits):
    bag = rosbag.Bag(bag_path)
    gps_msgs = []
    cloud_msgs = []

    all_times = []

    # Get all timestamps to determine duration
    for _, _, t in bag.read_messages(topics=['/gps_marker', '/mav_track/filtered_cloud']):
        all_times.append(t.to_sec())
    if not all_times:
        return []

    min_time = min(all_times)
    max_time = max(all_times)
    start_time = min_time + time_limits[0]
    end_time = max_time - time_limits[1]

    # Cut time
    for topic, msg, t in bag.read_messages(topics=['/gps_marker', '/mav_track/filtered_cloud']):
        t_sec = t.to_sec()
        if t_sec < start_time or t_sec > end_time:
            continue
        if topic == '/gps_marker':
            gps_msgs.append((t_sec, msg))
        elif topic == '/mav_track/filtered_cloud':
            cloud_msgs.append((t_sec, msg))

    cloud_times = np.array([t for t, _ in cloud_msgs])
    position_with_measurement = []

    for gps_time, gps_msg in gps_msgs:
        drone_pos = point_from_marker(gps_msg)
        if drone_pos is None:
            continue

        idx = np.argmin(np.abs(cloud_times - gps_time))

        _, cloud_msg = cloud_msgs[idx]
        cloud_points = pointcloud2_to_xyz_array(cloud_msg)

        distances = np.linalg.norm(cloud_points - np.array([drone_pos.x, drone_pos.y, drone_pos.z]), axis=1)
        count = np.sum(distances < gps_accuracy)
        if count == 0:
            distance_from_LiVOX = 0.0
        else:
            distance_from_LiVOX = np.linalg.norm([drone_pos.x, drone_pos.y, drone_pos.z])
        position_with_measurement.append((drone_pos, count, distance_from_LiVOX))

    bag.close()
    return position_with_measurement

def extract_axis(pos, axis):
    return getattr(pos, axis)

def main():
    
    bag_paths = [f"{bag_folder}/{bag}" for bag in bag_files]
    
    all_data = []
    for bag, time_limits in zip(bag_paths, bag_time_limits):
        csv_path = os.path.splitext(bag)[0] + ".csv"
        
        if os.path.exists(csv_path):
            print(f"Loading precomputed results from {csv_path}")
            data = pd.read_csv(csv_path).values.tolist()
        else:
            print(f"No precomputed results found for {bag}. Processing the bag file.")
            data = process_bag(bag, gps_accuracy, time_limits)
            # Save to CSV for future use
            df = pd.DataFrame(
                [
                    [pos.x, pos.y, pos.z, count, distance]
                    for pos, count, distance in data
                ],
                columns=['gps_x', 'gps_y', 'gps_z', 'count', 'distance']
            )
            
            df.to_csv(csv_path, index=False)
        
        all_data.append(data)
    
    if vert_color_bar:
        fig = plt.figure(figsize=(len(bag_paths) * 5, 5), constrained_layout=True)
        gs = gridspec.GridSpec(1, len(bag_paths) + 1, width_ratios=[1]*len(bag_paths) + [0.05], wspace=0.3)
        axes = [fig.add_subplot(gs[0, i]) for i in range(len(bag_paths))]
        # Get position of the leftmost and rightmost subplot
        pos0 = axes[0].get_position()
        pos1 = axes[-1].get_position()

        # Add colorbar axis manually, aligned with subplots
        cbar_ax = fig.add_axes([
            pos1.x1 + 0.01,      # x: just to the right of last subplot
            pos0.y0,             # y: align bottom with subplots
            0.015,               # width
            pos0.y1 - pos0.y0    # height: match subplot height
        ])
    else:
        fig = plt.figure(figsize=(len(bag_paths) * 5, 6), constrained_layout=True)
        gs = fig.add_gridspec(2, len(bag_paths), height_ratios=[1, 0.05], hspace=colorbar_spacing)

        axes = [fig.add_subplot(gs[0, i]) for i in range(len(bag_paths))]
        cbar_ax = fig.add_subplot(gs[1, :])  # colorbar spans full width
        

    all_counts = [count for data in all_data for _, _, _, count, _ in data]
    min_count = min(all_counts)
    max_count = max(all_counts)

    norm = plt.Normalize(vmin=min_count, vmax=max_count)
    cmap = plt.cm.viridis

    for i, (data, ax, axes_names) in enumerate(zip(all_data, axes, bag_axes)):
        x_axis, y_axis = axes_names
        xs = []
        ys = []
        for pos_x, pos_y, pos_z, _, _ in data:
            pos = Point(pos_x, pos_y, pos_z)
            xs.append(extract_axis(pos, x_axis))
            ys.append(extract_axis(pos, y_axis))
        
        distance_from_LiVOX = [distance for _, _, _, _, distance in data]
        max_distance = max(distance_from_LiVOX)
        idx_max_distance = distance_from_LiVOX.index(max_distance)
                
        counts = [count for _, _, _, count, _ in data]


        # Draw points with zero count first (red)
        red_xs = [x for x, count in zip(xs, counts) if count == 0]
        red_ys = [y for y, count in zip(ys, counts) if count == 0]
        ax.scatter(red_xs, red_ys, c='red', s=circle_size, label='No return')

        # Draw the rest
        rest_xs = [x for x, count in zip(xs, counts) if count > 0]
        rest_ys = [y for y, count in zip(ys, counts) if count > 0]
        rest_counts = [count for count in counts if count > 0]
        rest_colors = [cmap(norm(count)) for count in rest_counts]
        ax.scatter(rest_xs, rest_ys, c=rest_colors, s=circle_size)
        
        ax.set_title(f"{fig_names[i]} - Max Distance: {max_distance:.2f} $m$", fontsize=textsize)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True)
        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        max_range = max(x_range, y_range) * 1.1 / 2  # half range + 10%

        x_center = (max(xs) + min(xs)) / 2
        y_center = (max(ys) + min(ys)) / 2

        ax.set_xlim(x_center - max_range, x_center + max_range)
        ax.set_ylim(y_center - max_range, y_center + max_range)
        
        ax.set_xlabel(f"{x_axis.upper()} [$m$]", fontsize=textsize)
        ax.set_ylabel(f"{y_axis.upper()} [$m$]", fontsize=textsize)
        
        # Draw an arrow pointing to the point with the maximum distance
        ax.annotate(
            '', 
            xy=(xs[idx_max_distance], ys[idx_max_distance]), 
            xytext=(xs[idx_max_distance] + 0.5, ys[idx_max_distance] + 0.5),
            arrowprops=dict(facecolor='black', shrink=0.05, width=2, headwidth=8)
        )
        # Set tick size for both axes
        ax.tick_params(axis='both', which='major', labelsize=textsize)
        ax.tick_params(axis='both', which='minor', labelsize=textsize)


    if vert_color_bar:
        # Add colorbar to the far right of the figure
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
        sm.set_array([])
        cbar = plt.colorbar(sm, cax=cbar_ax)
        cbar.set_label(f'Points returned from UAV', fontsize=textsize)
        cbar.set_ticks([1, max_count])
        cbar.ax.tick_params(labelsize=textsize)
    else:
        # Create a custom colormap: red for min, then viridis for rest
        num_steps = max_count
        viridis_colors = plt.cm.viridis(np.linspace(0, 1, int(num_steps - 1)))
        custom_colors = np.vstack(([1.0, 0.0, 0.0, 1.0], viridis_colors))
        custom_cmap = ListedColormap(custom_colors)

        # Adjust norm to align red with minimum
        norm = plt.Normalize(vmin=0, vmax=max_count)

        # Create scalar mappable with the custom colormap
        sm = plt.cm.ScalarMappable(cmap=custom_cmap, norm=norm)
        sm.set_array([])

        # Add colorbar
        cbar = plt.colorbar(sm, cax=cbar_ax, orientation='horizontal')
        cbar.set_label('Points returned from UAV',  fontsize=textsize)

        cbar.set_ticks([0] + list(np.linspace(1, max_count, 6, dtype=int)))
        cbar.ax.tick_params(labelsize=textsize)

    plt.savefig(output_figure_name)
    plt.show()

if __name__ == "__main__":
    main()
