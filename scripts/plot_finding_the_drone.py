import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import *
import pandas as pd

textsize=35
linewidth=3

# just to test the script: 
# data_path = "/home/appuser/livox_mav_track/scripts/results/accuracy/TEST1_horizontal_KF_Dogru_Use_Min_Search_Radius_true_Tracking_Type_1_KF_Motion_Model_Type_0_KF_Search_Radius_0.12_run1.csv"
# the one in the paper:
data_path="/home/appuser/livox_mav_track/scripts/results/accuracy/TEST5_horizontal_Tracking_Type_0_ParticleFilter_Motion_Model_Type_1_run3.csv"
output_figure_name="/home/appuser/livox_mav_track/scripts/figs/finding_the_drone.png"  
maxtime_s=10
mintime_s=2

def main():   
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
    
    fig = plt.figure(figsize=(20,12))
    ax1 = fig.add_subplot()
    ax1.plot(((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000, data['distance_error'], color=color1, linewidth=linewidth, label="PF error")    
    
    ax1.set_xlabel("Time $[s]$",fontsize=textsize)
    ax1.set_ylabel("Distance $[m]$",fontsize=textsize)
    
    # ax2 = ax1.twinx()
    ax1.plot(((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000, data['particles_avstd'], color=color3, linestyle='--', alpha=1.0, linewidth=linewidth, label="$\\sigma_{particles}$")
    ax1.axhline(0.09, color=color2, linestyle='-.',linewidth=linewidth, label='$\\sigma_{threshold}=0.09$')
    
    
    # Create legend handles for the plots you want to include in the legend
    handles1, labels1 = ax1.get_legend_handles_labels()
    plt.legend(handles1, labels1, fontsize=textsize, loc='upper right')
    ax1.locator_params(axis='y', nbins=4)
    
    ax1.tick_params(labelsize=textsize)

    plt.rcParams['text.usetex'] = True
    fig.savefig(output_figure_name)
    plt.show()
       
    return
    
    
if __name__ == '__main__':
    main()