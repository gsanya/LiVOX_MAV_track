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

textsize=35
linewidth=2

# just to test the script: 
# data_path = "/home/appuser/livox_mav_track/scripts/results/accuracy/TEST1_horizontal_KF_Dogru_Use_Min_Search_Radius_true_Tracking_Type_1_KF_Motion_Model_Type_0_KF_Search_Radius_0.12_run1.csv"
# the one in the paper:
data_path = "/home/appuser/livox_mav_track/scripts/results/accuracy/TEST5_horizontal_Tracking_Type_0_ParticleFilter_Motion_Model_Type_1_run1.csv"
output_figure_name="/home/appuser/livox_mav_track/scripts/figs/horiz_accuracy.png"
maxtime_s=41
mintime_s=8


def main():    
    print(f"loading csv: {data_path}")
    data=pd.read_csv(data_path)

    print(data)
          
    # cut by time
    data = data[((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000 < maxtime_s]
    data = data[((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000 > mintime_s]
    data = data.reset_index(drop=True)       
  
    #%% figure
    color1='darkorange'
    color2='seagreen'
    color3='rebeccapurple'
    color4='firebrick'
    
    stdev=np.std(data['distance_error'])
    mean=np.mean(data['distance_error'])
    
    # calculate rmse
    rmse=0.0
    for index, row in data.iterrows():
        rmse+=row['error_x']**2+row['error_y']**2+row['error_y']**2
    length=data.shape[0]
    rmse=math.sqrt(rmse/length)
    
    print(f"mean: {mean}\nrmse: {rmse}")
    
    fig = plt.figure(figsize=(20,12))
    ax1 = fig.add_subplot()
    ax1.plot(((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000, data['distance_error'], color=color1, linewidth=linewidth, label="PF error")
    ax1.set_xlabel("Time $[s]$",fontsize=textsize)
    ax1.set_ylabel("Error $[m]$",fontsize=textsize)
    
    meanlabel="$\\mu_{error}="+f"{mean:.3f}"+"$"
    stdlabel="$\\sigma_{error}="+f"{stdev:.3f}"+"$"
    ax1.axhline(mean, color=color4, linestyle='--', label=meanlabel,linewidth=linewidth)
    ax1.axhline(mean+stdev, color=color2, linestyle=':', label=stdlabel,linewidth=linewidth)
    ax1.axhline(mean-stdev, color=color2, linestyle=':', linewidth=linewidth)
    
    ax2 = ax1.twinx()
    ax2.plot(((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000, data['GT_speed'], color=color3, linestyle='--', alpha=1.0, linewidth=linewidth, label="GT speed")
    ax2.set_ylabel("GT speed $[m/s]$",fontsize=textsize)
    
    # Create legend handles for the plots you want to include in the legend
    handles1, labels1 = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    handles=handles1+handles2
    labels=labels1+labels2
    ax1.tick_params(labelsize=textsize)
    ax2.tick_params(labelsize=textsize)
    ax1.locator_params(axis='x', nbins=10)
    ax1.locator_params(axis='y', nbins=4)
    ax2.locator_params(axis='y', nbins=4)
    
    # Add legend
    plt.legend(handles, labels, fontsize=textsize, loc='upper left')
    # fig.legend(handles, labels, fontsize=textsize)
    
    
    fig.savefig(output_figure_name)
    plt.show()
    
    
    return

if __name__ == '__main__':
    main()