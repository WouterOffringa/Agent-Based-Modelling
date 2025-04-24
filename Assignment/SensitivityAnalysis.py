import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Set the backend to 'TkAgg' (interactive)
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import re

## What to plot
means_per_run = True #True or False
local = True
taxibot = False
spawning = True
extended_range = True


def load_simulation_results(filename, case):
    filepath = "./Gebeunde_sensitivity_results/" + case + filename
    try:
        data = pd.read_csv(filepath)
        return data
    except FileNotFoundError:
        print(f"Error: The file {filename} was not found in the simulation_results folder.")
        return None

# Define cases for local/global
if local:
    if taxibot:
        cases = ['/taxibot_3/', '/taxibot_4/', '/taxibot_5/']
        p_list = [3, 4, 5]

    if spawning:
        if not extended_range:
            cases = ['/spawn_3.5/', '/spawn_4/', '/spawn_4.5/']
            p_list = [3.5, 4, 4.5]
        if extended_range:
            cases = ['/spawn_3/','/spawn_3.5/','/spawn_4/' , '/spawn_4.5/', '/spawn_5/']
            p_list = [3, 3.5, 4, 4.5, 5]


if not local:
    spawn_vals = ['3', '3.5', '4']
    taxibot_vals = ['3', '4', '5']
    cases = []
    cases += [f'/spawn{sp}_taxibot{tb}_v1/' for sp in spawn_vals for tb in taxibot_vals]


mean_delays = []
std_delays = []
all_delays_matrix = []


if means_per_run:
    case_delays = {}

    for case in cases:
        all_delays = []

        for i in range(1, 6):
            filename = f"simulation_results_{i}.csv"
            print(f"Currently checking case {case} and file {filename}")
            data = load_simulation_results(filename, case)
            if data is not None:
                if 'total_delay' in data.columns:
                    delays = data['total_delay'].dropna().tolist()
                    print(f"  Found {len(delays)} valid delays in {filename}")
                    all_delays.extend(delays)
                else:
                    print(f"  Warning: 'total_delay' column not found in {filename}")

        print(f"Total delays collected for case {case}: {len(all_delays)}")
        if len(all_delays) < 100:
            print('Too little datapoints for', f"Total delays collected for case {case}" )
        all_delays_matrix.append(all_delays)
        if all_delays:
            case_delays[case] = all_delays
        else:
            print(f"  No valid delays found for case {case}")

    print("\n===== Case Statistics =====")
    for case, delays in case_delays.items():
        mean_delay = np.mean(delays)
        mean_delays.append(mean_delay)
        std_delay = np.std(delays)
        std_delays.append(std_delay)
        print(f"{case}: mean = {mean_delay:.2f}, std = {std_delay:.2f}")

if local:
    if taxibot:
        fig, ax = plt.subplots(figsize=(8, 6))

        # Create the boxplot
        for x, delays in zip(p_list, all_delays_matrix):
            ax.boxplot(delays, positions=[x], widths=0.2, patch_artist=True,
                       boxprops=dict(facecolor='lightblue', linewidth=2),
                       medianprops=dict(color='darkblue', linewidth=2),
                       whiskerprops=dict(linewidth=2),
                       capprops=dict(linewidth=2))

        # Line and dots for means — using exact same x positions
        ax.plot(p_list, mean_delays, color='red', linestyle='-', linewidth=2)
        ax.scatter(p_list, mean_delays, color='black', zorder=5, label='Mean')

        # Set labels and title
        plt.xlabel("Number of Taxibots", fontsize=14)
        plt.ylabel("Delay(s)", fontsize=14)
        plt.title("Delay vs Number of Taxibots", fontsize=16)
        plt.legend(loc='upper right', fontsize=12)
        ax.tick_params(axis='both', labelsize=14)

        # Customize grid and layout
        plt.grid(True)
        plt.tight_layout()

        # Show the plot
        plt.show()

    if spawning:
        fig, ax = plt.subplots(figsize=(8, 6))

        # Create the boxplot
        for x, delays in zip(p_list, all_delays_matrix):
            ax.boxplot(delays, positions=[x], widths=0.2, patch_artist=True,
                       boxprops=dict(facecolor='lightblue', linewidth=2),
                       medianprops=dict(color='darkblue', linewidth=2),
                       whiskerprops=dict(linewidth=2),
                       capprops=dict(linewidth=2))

        # Line and dots for means — using exact same x positions
        ax.plot(p_list, mean_delays, color='red', linestyle='-', linewidth=2)
        ax.scatter(p_list, mean_delays, color='black', zorder=5, label='Mean')

        # Set labels and title
        plt.xlabel("Spawning time (s)", fontsize=14)
        plt.ylabel("Delay(s)", fontsize=14)
        plt.title("Delay vs spawning time", fontsize=16)
        plt.legend(loc='upper right', fontsize=12)
        ax.tick_params(axis='both', labelsize=14)

        # Customize grid and layout
        plt.grid(True)
        plt.tight_layout()

        # Show the plot
        plt.show()


if not local:
    heatmap_matrix_v1 = np.array([
        [mean_delays[6], mean_delays[7], mean_delays[8]],  # spawn = 4
        [mean_delays[3], mean_delays[4], mean_delays[5]],  # spawn = 3.5
        [mean_delays[0], mean_delays[1], mean_delays[2]],  # spawn = 3
    ])

    standard_matrix_v1 = np.array([
        [std_delays[6], std_delays[7], std_delays[8]],  # spawn = 4
        [std_delays[3], std_delays[4], std_delays[5]],  # spawn = 3.5
        [std_delays[0], std_delays[1], std_delays[2]],  # spawn = 3
    ])

    x_labels = [3, 4, 5]  # Number of taxibots (X-axis)
    y_labels = [3, 3.5, 4]  # Spawn times (Y-axis)

    # Plot
    plt.figure(figsize=(16, 6))

    # Set global font size
    plt.rcParams.update({'font.size': 14})

    # First heatmap
    plt.subplot(1, 2, 1)
    sns.heatmap(
        heatmap_matrix_v1,
        annot=True,
        fmt=".2f",
        cmap="Blues",
        cbar=True,
        annot_kws={"size": 14},  # Annotation inside heatmap
        xticklabels=x_labels,
        yticklabels=y_labels
    )
    plt.xlabel('Number of taxibots', fontsize=14)
    plt.ylabel('Spawn time (s)', fontsize=14)
    plt.title('Mean delay (s)', fontsize=16)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)

    # Second heatmap
    plt.subplot(1, 2, 2)
    sns.heatmap(
        standard_matrix_v1,
        annot=True,
        fmt=".2f",
        cmap="Reds",
        cbar=True,
        annot_kws={"size": 14},
        xticklabels=x_labels,
        yticklabels=y_labels
    )
    plt.xlabel('Number of taxibots', fontsize=14)
    plt.ylabel('Standard deviation', fontsize=14)
    plt.title('Standard deviation', fontsize=16)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)

    plt.tight_layout()
    plt.show()