import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


#What to plot?

means_per_run = False #True or False
All_delays = True

def load_simulation_results(filename):
    filepath = "Results/Sim_Results_with_path_lengthsss/" + filename
    try:
        data = pd.read_csv(filepath)
        return data
    except FileNotFoundError:
        return None

#Means per run
if means_per_run == True:
    All_means = []
    All_stds = []

    for i in range(110):
        filename = f"simulation_results_{i}_Collisions0.csv"
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

    alfa = 0.05 #95% confidence interval
    l = 0.5 #length of the interval for the total delay
    z_value = 1.96 #fill in compared to alfa /2 (alpha 0.05, gives alfa/2 of 0.025, z_value = 1.96)
    X_i_mean = 0 #mean of the total delay
    S_i = 0 #standard deviation of the total delay    
    min_data = 90


    for i in range(len(All_means)):
        print("Run: ", i)
        X_i_mean_new = X_i_mean + (All_means[i] - X_i_mean)/(i+1+1)
        S_i = (1 - 1/(i+1)) * S_i**2 + (i+1+1)* (X_i_mean_new - X_i_mean)**2
        X_i_mean = X_i_mean_new
        S_i = np.sqrt(S_i)
        bound = 2 * z_value * S_i/(np.sqrt(i+1))
        print("Bound: ", bound)
        if i>min_data and i>= 99:          
            if bound < l:
                print("Confidence interval is smaller than the length of the interval")
                break

if All_delays == True:

    Delays = []
    Waiting_times = []
    Taxi_delays = []
    Number_of_sims = 0
    for i in range(108):
        filename = f"simulation_results_{i}_Collisions0.csv"
        data = load_simulation_results(filename)
        if data is not None:
            Number_of_sims += 1
            print("Number of simulations: ", Number_of_sims)
            print(filename)
            #print(filename, "has number of ac: ", len(data['total_delay']))

            Delays.extend(data['total_delay'].tolist())  # Append all delays to the list
            Waiting_times.extend(data['waiting_time'].tolist())
            Taxi_delays.extend(data['taxi_delay'].tolist())
            

    Delays = np.array(Delays)  # Convert to numpy array for easier manipulation
    No_of_Aircraft = len(Delays)  # Total number of aircrafts
    mean_delay = np.mean(Delays)  # Calculate the mean of the delays
    median_delay = np.median(Delays)  # Calculate the median of the delays
    Sum_of_Waiting_times = np.sum(Waiting_times)  # Calculate the sum of the waiting times
    Sum_of_Taxi_delays = np.sum(Taxi_delays)  # Calculate the sum of the taxi delays
    Sum_of_delays = np.sum(Delays)  # Calculate the sum of the taxi delays
    Standard_deviation = np.std(Delays)  # Calculate the standard deviation of the delays

    Percentage_of_Waiting_times = (Sum_of_Waiting_times / Sum_of_delays) * 100  # Calculate the percentage of waiting times
    Percentage_of_Taxi_delays = (Sum_of_Taxi_delays / Sum_of_delays) * 100  # Calculate the percentage of taxi delays
    print(f"percentage of waiting times: {Percentage_of_Waiting_times}%")
    print(f"percentage of taxi delays: {Percentage_of_Taxi_delays}%")
    print(f"Standard deviation of the delays: {Standard_deviation}")

    print(f"The average delay is: {mean_delay}")
    print(f"Median delay: {median_delay}")
    #plot the delays
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    bins = np.arange(min(Delays), max(Delays) + 1, 1)  # Bin edges with size 1
    plt.hist(Delays, bins=bins, color='blue', alpha=0.7)
    plt.title(f'Delays of aircrafts, Number of AC:{No_of_Aircraft} in {Number_of_sims} simulations')
    plt.xlabel('Total Delay [s]')
    plt.ylabel('Frequency')
    plt.grid()
    plt.show()

    #Create a boxplot of the total delays
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 2)
    plt.boxplot(Delays, positions=[0.5], widths=0.2, patch_artist=True,
                boxprops=dict(facecolor='lightblue', linewidth=2),
                medianprops=dict(color='darkblue', linewidth=2),
                whiskerprops=dict(linewidth=2),
                capprops=dict(linewidth=2))
    plt.scatter(0.5, mean_delay, color='red', zorder=3, label='Mean', s=50)  # Add a red dot for the mean
    plt.title(f'Boxplot of total delays, Number of AC:{No_of_Aircraft} in {Number_of_sims} simulations')
    plt.xlabel('Total Delay [s]')
    plt.legend()
    plt.grid()  
    plt.show()

    #Calculate the confidence intervals

    alfa = 0.05 #95% confidence interval
    l = 0.5 #length of the interval for the total delay
    z_value = 1.96 #fill in compared to alfa /2 (alpha 0.05, gives alfa/2 of 0.025, z_value = 1.96)
    X_i_mean = 0 #mean of the total delay
    S_i = 0 #standard deviation of the total delay    
    min_data = 2300


    for i in  range(len(Delays)):
        X_i_mean_new = X_i_mean + (Delays[i] - X_i_mean)/(i+1+1)
        S_i = (1 - 1/(i+1)) * S_i**2 + (i+1+1)* (X_i_mean_new - X_i_mean)**2
        X_i_mean = X_i_mean_new
        S_i = np.sqrt(S_i)
        bound = 2 * z_value * S_i/(np.sqrt(i+1))
        if i>min_data:           
            if bound < l:
                print("Confidence interval is smaller than the length of the interval")
                break
