import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

textsize=15
circle_size=100
star_size=400
colorbar_spacing = 0.1

data_path = "/home/appuser/livox_mav_track/scripts/results/grid_search_stds/summary_results.csv"
output_figure_name = "/home/appuser/livox_mav_track/scripts/figs/param_grid_plot.png"


def main():
    df = pd.read_csv(data_path)
        
    # only plot those points, that av_max_dist is at least 120 and av_tracking_pct is at least 55%
    df = df[(df['av_max_dist'] >= 120) & (df['av_tracking_pct'] >= 55)]

    # Define baseline parameters
    baseline = {
        'std_pos': 0.5,
        'std_vel': 0.5,
        'std_acc': 0.0,
        'std_sense': 0.2
    }   

    # Compute Euclidean distance from baseline
    df['param_change'] = np.sqrt(
        (df['std_pos'] - baseline['std_pos'])**2 +
        (df['std_vel'] - baseline['std_vel'])**2 +
        (df['std_acc'] - baseline['std_acc'])**2 +
        (df['std_sense'] - baseline['std_sense'])**2
    )

    # Separate the baseline point from the rest
    baseline_mask = (
        (df['std_pos'] == baseline['std_pos']) &
        (df['std_vel'] == baseline['std_vel']) &
        (df['std_acc'] == baseline['std_acc']) &
        (df['std_sense'] == baseline['std_sense'])
    )
    
    df_baseline = df[baseline_mask]
    df_others = df[~baseline_mask]

    # Plot
    plt.figure(figsize=(10, 7))

    # First, plot all the non-baseline points
    scatter = plt.scatter(
        df_others['av_max_dist'],
        df_others['av_tracking_pct'],
        c=df_others['param_change'],
        cmap='viridis',
        s=circle_size,
        edgecolor='k',
        label='Perturbed configs'
    )

    # Then, plot the baseline on top in red
    plt.scatter(
        df_baseline['av_max_dist'],
        df_baseline['av_tracking_pct'],
        color='red',
        s=star_size,
        edgecolor='k',
        marker='*',
        label='Baseline'
    )

    # Labels and colorbar
    plt.xlabel('Maximum Tracking Distance', fontsize=textsize)
    plt.ylabel('Tracking Coverage', fontsize=textsize)

    cbar = plt.colorbar(scatter)
    cbar.set_label('Parameter Change Magnitude', fontsize=textsize)
    cbar.ax.tick_params(labelsize=textsize)

    plt.tick_params(axis='both', which='major', labelsize=textsize)
    plt.tick_params(axis='both', which='minor', labelsize=textsize)
    
    plt.legend(fontsize=textsize)
    plt.grid(True)
    plt.tight_layout()
    
    plt.savefig(output_figure_name)
    plt.show()

    


if __name__ == "__main__":
    main()
