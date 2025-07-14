import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
params = {'legend.fontsize': 'xx-large',
          'figure.figsize': (20, 12),
         'axes.labelsize': 'xx-large',
         'axes.titlesize':'xx-large',
         'xtick.labelsize':'xx-large',
         'ytick.labelsize':'xx-large'}
pylab.rcParams.update(params)
import numpy as np
import pandas as pd
from mpl_toolkits import mplot3d
import rosbag
import math
import os
import copy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

max_cloud_distance=1.0
max_velocity=7.0
sigma_threshold=0.75

# # sunny case
# data_path='/home/appuser/livox_mav_track/scripts/results/field_test/sunny_result.csv'
# # only needed if csv doesn't exists yet
# bag_path='/home/appuser/livox_mav_track/scripts/results/field_test/sunny_result.bag'
# figure_3D_file_name='/home/appuser/livox_mav_track/scripts/figs/3Dplot_sunny.png'
# # the first plot helps a lot to determine where the drone was lost
# maxtime_s=90
# mintime_s=10
# # camera params
# elev=40
# azim=-115
# xticks=[20,40,60,80,100,120]
# yticks=[-5,5]
# zticks=[10,20]
# xlabelpad=120
# ylabelpad=20
# zlabelpad=10

# foggy case
data_path='/home/appuser/livox_mav_track/scripts/results/field_test/foggy_result.csv'
# only needed if csv doesn't exists yet
bag_path='/home/appuser/livox_mav_track/scripts/results/field_test/foggy_result.bag'
figure_3D_file_name='/home/appuser/livox_mav_track/scripts/figs/3Dplot_foggy.png'
# the first plot helps a lot to determine where the drone was lost
maxtime_s=95
mintime_s=40
# camera params
elev=30
azim=245
xticks=[10,20,30,40,50]
yticks=[-5,0,5]
zticks=[0,5,10]
xlabelpad=60
ylabelpad=10
zlabelpad=10


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
   
def main(): 
	column_names = ['x', 'y', 'z', 'speed', 'distance', 'time_s', 'time_ns', 'particles_avstd', 'cloud_size_1m']
	data = pd.DataFrame(columns=column_names)
		
	if not os.path.exists(data_path):
		print("creating csv")
		bag = rosbag.Bag(bag_path, 'r')
		
		last_time_second = 0
		last_time_nano = 0
		start_time_second = 0
		start_time_nano = 0
		last_position_x = 0
		last_position_y = 0
		last_position_z = 0
		first = True
		cloud_size = 0
		
		max_distance = 0.0
		particles_avstd = 0.1
		# load data from bag
		drone_pose_stamped=PoseStamped()
		data_list = []
		
		for topic, msg, t in bag.read_messages(topics=["/mav_track/mav_marker","/mav_track/particles","/mav_track/filtered_cloud"]):
			if topic=="/mav_track/mav_marker":
				if msg.ns=="mav_model":
					continue
			
				# convert marker to posestamped
				drone_pose_stamped=marker_to_posestamped(msg)
    
				msg=copy.deepcopy(drone_pose_stamped)
				
				# max distance search
				distance = math.sqrt(msg.pose.position.x**2+msg.pose.position.y**2+msg.pose.position.z**2)
				if distance > max_distance:
					max_distance = distance

				if not first:
					delta_time_s = msg.header.stamp.secs - last_time_second
					delta_time_ns = msg.header.stamp.nsecs - last_time_nano
					full_delta_time = delta_time_s + delta_time_ns / 1000000000.0
					
     				# avoid division by zero
					if full_delta_time == 0:
						full_delta_time = 0.000001  
      
					delta_distance = math.sqrt((msg.pose.position.x - last_position_x)**2+ (msg.pose.position.y - last_position_y)**2 +(msg.pose.position.z - last_position_z)**2)
					velocity = delta_distance / full_delta_time
					
					if velocity > max_velocity:
						print("velocity too high")
						continue
					
					new_data = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, velocity, distance, msg.header.stamp.secs, msg.header.stamp.nsecs, particles_avstd, cloud_size]
					data_list.append(pd.Series(new_data, index=data.columns))
			
				else:
					start_time_second=msg.header.stamp.secs
					start_time_nano=msg.header.stamp.nsecs    
					new_data = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 0, distance, msg.header.stamp.secs, msg.header.stamp.nsecs, particles_avstd, cloud_size]
					data_list.append(pd.Series(new_data, index=data.columns))
					first=False
					
				last_time_second=msg.header.stamp.secs
				last_time_nano=msg.header.stamp.nsecs
				last_position_x=msg.pose.position.x
				last_position_y=msg.pose.position.y
				last_position_z=msg.pose.position.z
		
			if topic=="/mav_track/particles":
				x_coordinates = [point.x for point in msg.points]
				y_coordinates = [point.y for point in msg.points]
				z_coordinates = [point.z for point in msg.points]
				
				std_dev_x = np.std(x_coordinates) 
				std_dev_y = np.std(y_coordinates) 
				std_dev_z = np.std(z_coordinates) 
				particles_avstd = (std_dev_x+std_dev_y+std_dev_z)/3
			
			if topic=="/mav_track/filtered_cloud":
				# only count points that are max max_cloud_distance m from drone pose
				if drone_pose_stamped!=PoseStamped():
					cloud_size = 0
				for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
					distance = math.sqrt((drone_pose_stamped.pose.position.x-point[0])**2 +
										(drone_pose_stamped.pose.position.y-point[1])**2 +
										(drone_pose_stamped.pose.position.z-point[2])**2)
					if distance < max_cloud_distance:
						cloud_size += 1
					else:
						print("too far from position")
		data = pd.DataFrame(data_list)    
		data.to_csv(data_path, index=False)
	else:
		print("loading csv")
		data=pd.read_csv(data_path)  
		start_time_second=data["time_s"][0]
		start_time_nano=data["time_ns"][0]
	
	
	print(data)
	
	# cut by mintime
	data = data[((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000 > mintime_s]
	data = data.reset_index(drop=True)
	
	print(data)
 
	# calculate maxtime
	start_sec=data["time_s"][0]
	start_nsec=data["time_ns"][0]
	# create times and tracked lists
	tracked=[]
	times=[]
	for index, row in data.iterrows():       
		times.append(((data["time_s"][index]-start_sec)*1000000000+data["time_ns"][index]-start_nsec)/1000000000)
		if row['particles_avstd']<sigma_threshold:
			tracked.append(1)
		else:
			tracked.append(0)
				
	# find the point when the drone was tracked less than 1 times in 10
	for index in range(len(tracked)):
		if index < len(tracked)-10:
			sum=0
			for ii in range(10):
				sum+=tracked[index+ii]
			if sum<1:
				maxtime_s=times[index]
				break
	
	
	# cut by maxtime              
	data = data[((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000 < maxtime_s]
	data = data.reset_index(drop=True)
	
	print(data)
	
	# clean up based on std of particles (if its big, the tracking was lost)
	data = data[abs(data['particles_avstd']) < sigma_threshold]
	data = data.reset_index(drop=True)
 
	print(data)		
		
	#%% 3D plot with speed (only plot when MAV was tracked)
	fig = plt.figure()
	ax = fig.add_subplot(projection='3d')
	ax.view_init(elev=elev, azim=azim)
	p = ax.scatter(data['x'], data['y'], data['z'], c=data['speed'], cmap='viridis', s=50)
	
	# ax.get = lambda: np.dot(mplot3d.Axes3D.get_proj(ax), np.diag([x_scale, y_scale, z_scale, 1]))
	
	fig.colorbar(p, shrink=0.4, label="speed $[m/s]$", pad=0.01)

	ax.set_xlabel('X $[m]$', labelpad=xlabelpad)
	ax.set_ylabel('Y $[m]$', labelpad=ylabelpad)
	ax.set_zlabel('Z $[m]$', labelpad=zlabelpad)
	
	x_limits = ax.get_xlim3d()
	y_limits = ax.get_ylim3d()
	z_limits = ax.get_zlim3d()
	ax.set_box_aspect([x_limits[1]-x_limits[0],y_limits[1]-y_limits[0],z_limits[1]-z_limits[0]])

	ax.set_xticks(xticks)
	ax.set_yticks(yticks)
	ax.set_zticks(zticks)

	#ax.grid()
	fig.savefig(figure_3D_file_name)
	plt.show()
	print('end')
	
if __name__ == '__main__':
	main()