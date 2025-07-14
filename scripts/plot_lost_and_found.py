import matplotlib.pyplot as plt
from tf.transformations import *
import pandas as pd
    
textsize=35
linewidth=3
    
# Finding the drone
# just to test the script: 
# data_path_PF="/home/appuser/livox_mav_track/scripts/results/accuracy/TEST1_horizontal_KF_Dogru_Use_Min_Search_Radius_true_Tracking_Type_1_KF_Motion_Model_Type_0_KF_Search_Radius_0.12_run1.csv"
# data_path_EKF="/home/appuser/livox_mav_track/scripts/results/accuracy/TEST1_horizontal_KF_Dogru_Use_Min_Search_Radius_true_Tracking_Type_1_KF_Motion_Model_Type_0_KF_Search_Radius_0.12_run5.csv"
# the one in the paper:
data_path_PF="/home/appuser/livox_mav_track/scripts/results/accuracy/TEST5_lostandfound_Tracking_Type_0_ParticleFilter_Motion_Model_Type_1_run1.csv"
data_path_EKF="/home/appuser/livox_mav_track/scripts/results/accuracy/TEST2_lostandfound_KF_Dogru_Use_Min_Search_Radius_false_Tracking_Type_1_KF_Motion_Model_Type_0_KF_Search_Radius_0.12_run1.csv"
output_figure_name="/home/appuser/livox_mav_track/scripts/figs/lost_and_found_PF_EKF2.svg"  

# only one loss
maxtime_s=100
mintime_s=0
    
def main():

    data=pd.read_csv(data_path_PF)
    data2=pd.read_csv(data_path_EKF)
    
    print(data)
    
    # cut by time
    data = data[((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000 < maxtime_s]
    data = data[((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000 > mintime_s]
    data = data.reset_index(drop=True)
    
    data2 = data2[((data2["time_s"]-data2["time_s"][0])*1000000000+data2["time_ns"]-data2["time_ns"][0])/1000000000 < maxtime_s]
    data2 = data2[((data2["time_s"]-data2["time_s"][0])*1000000000+data2["time_ns"]-data2["time_ns"][0])/1000000000 > mintime_s]
    data2 = data2.reset_index(drop=True)
    
    #%% figure
    color1='darkorange'
    color2='seagreen'
    color3='rebeccapurple'
    color4='firebrick'
        
    fig = plt.figure(figsize=(20,12))
    ax1 = fig.add_subplot()
    ax1.plot(((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000, data['distance_error'], color=color1, linewidth=linewidth, label="PF error")
    ax1.plot(((data2["time_s"]-data2["time_s"][0])*1000000000+data2["time_ns"]-data2["time_ns"][0])/1000000000, data2['distance_error'], color=color4, linestyle=':', linewidth=linewidth, label="EKF error")
    
    ax1.set_xlabel("Time $[s]$",fontsize=textsize)
    ax1.set_ylabel("Distance $[m]$",fontsize=textsize)
    
    # ax2 = ax1.twinx()
    ax1.plot(((data["time_s"]-data["time_s"][0])*1000000000+data["time_ns"]-data["time_ns"][0])/1000000000, data['particles_avstd'], color=color3, linestyle='--', alpha=1.0, linewidth=linewidth, label="$\\sigma_{particles}$")
    ax1.axhline(0.09, color=color2, linestyle='-.',linewidth=linewidth, label='$\\sigma_{threshold}=0.09$')
    
    # ax2.set_ylabel("Average $\\sigma[-]$",fontsize=textsize)
    
    # Create legend handles for the plots you want to include in the legend
    handles1, labels1 = ax1.get_legend_handles_labels()
    # handles2, labels2 = ax2.get_legend_handles_labels()
    # handles=handles1+handles2
    # labels=labels1+labels2
    plt.legend(handles1, labels1, fontsize=textsize, loc='upper left')
    ax1.locator_params(axis='y', nbins=4)
    # ax2.locator_params(axis='y', nbins=4)
    
    ax1.tick_params(labelsize=textsize)
    # ax2.tick_params(labelsize=textsize)
    
    plt.rcParams['text.usetex'] = True
    fig.savefig(output_figure_name)
    plt.show()
    
    
    return
    
    
if __name__ == '__main__':
    main()