import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


#What to plot?

means_per_run = False #True or False
All_delays = True

def load_simulation_results(filename):
    filepath = "./simulation_results/" + filename
    try:
        data = pd.read_csv(filepath)
        return data
    except FileNotFoundError:
        print(f"Error: The file {filename} was not found in the simulation_results folder.")
        return None

#Means per run
if means_per_run == True:
    All_means = []
    All_stds = []

    for i in range(108):
        filename = f"simulation_results_{i}.csv"
        data = load_simulation_results(filename)
        if data is not None:
            Mean = data['total_delay'].mean()
            All_means.append(Mean)
            Std = data['total_delay'].std()
            All_stds.append(Std)



    # Convert to numpy arrays for easier manipulation
    All_means = np.array(All_means)
    All_stds = np.array(All_stds)

    #Create histogram plots of the means and standard deviations

    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.hist(All_means, bins=10, color='blue', alpha=0.7)
    plt.title('Histogram of Means total delay per run')
    plt.xlabel('Mean Total Delay')
    plt.ylabel('Frequency')
    plt.grid()
    plt.show()

if All_delays == True:

    All_delays = []

    for i in range(108):
        filename = f"simulation_results_{i}.csv"
        data = load_simulation_results(filename)
        if data is not None:
            All_delays.extend(data['total_delay'].tolist())  # Append all delays to the list

    All_delays = np.array(All_delays)  # Convert to numpy array for easier manipulation

    #plot the delays
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    bins = np.arange(min(All_delays), max(All_delays) + 1, 1)  # Bin edges with size 1
    plt.hist(All_delays, bins=bins, color='blue', alpha=0.7)
    plt.title('Histogram of All delays')
    plt.xlabel('Total Delay')
    plt.ylabel('Frequency')
    plt.grid()
    plt.show()

    #Calculate the confidence intervals
    
