import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the data from the uploaded file
file_path = '../tasks-nc.csv'
data = pd.read_csv(file_path)

# Calculate the average task difficulty for each task
average_difficulty = data.groupby('Task ID')[
    ['Task difficulty - Kinesthetic teaching', 'Task difficulty - Teleoperation']].mean()
average_difficulty.loc['Overall'] = data[
    ['Task difficulty - Kinesthetic teaching', 'Task difficulty - Teleoperation']].mean()

# Calculate the standard deviation for each task
std_difficulty = data.groupby('Task ID')[
    ['Task difficulty - Kinesthetic teaching', 'Task difficulty - Teleoperation']].std()
std_difficulty.loc['Overall'] = data[
    ['Task difficulty - Kinesthetic teaching', 'Task difficulty - Teleoperation']].std()

# Resetting the index to avoid KeyErrors
average_difficulty = average_difficulty.reset_index()
std_difficulty = std_difficulty.reset_index()

# Plotting the bar chart
plt.figure(figsize=(12, 6))  # Adjust the height to make the bars shorter
bar_width = 0.35
index = np.arange(len(average_difficulty))

# Plot bars for kinesthetic teaching
plt.bar(index, average_difficulty['Task difficulty - Kinesthetic teaching'], bar_width, label='Kinesthetic Teaching',
        yerr=std_difficulty['Task difficulty - Kinesthetic teaching'], capsize=5)

# Plot bars for teleoperation
plt.bar(index + bar_width, average_difficulty['Task difficulty - Teleoperation'], bar_width, label='Teleoperation',
        yerr=std_difficulty['Task difficulty - Teleoperation'], capsize=5)

# Add labels for mean and standard deviation
for i in range(len(average_difficulty)):
    plt.text(index[i], average_difficulty['Task difficulty - Kinesthetic teaching'][i],
             f'Mean: {average_difficulty["Task difficulty - Kinesthetic teaching"][i]:.2f}',
             horizontalalignment='center', verticalalignment='bottom', color='red', fontsize=10)
    plt.text(index[i], average_difficulty['Task difficulty - Kinesthetic teaching'][i] +
             std_difficulty['Task difficulty - Kinesthetic teaching'][i],
             f'Std: {std_difficulty["Task difficulty - Kinesthetic teaching"][i]:.2f}', horizontalalignment='center',
             verticalalignment='bottom', color='blue', fontsize=10)

    plt.text(index[i] + bar_width, average_difficulty['Task difficulty - Teleoperation'][i],
             f'Mean: {average_difficulty["Task difficulty - Teleoperation"][i]:.2f}', horizontalalignment='center',
             verticalalignment='bottom', color='red', fontsize=10)
    plt.text(index[i] + bar_width, average_difficulty['Task difficulty - Teleoperation'][i] +
             std_difficulty['Task difficulty - Teleoperation'][i],
             f'Std: {std_difficulty["Task difficulty - Teleoperation"][i]:.2f}', horizontalalignment='center',
             verticalalignment='bottom', color='blue', fontsize=10)

# Set the y-axis limits to make the bars shorter
plt.ylim(0, 12)
plt.yticks(np.arange(0, 13, 2))

# Add labels, title, and legend
plt.xlabel('Task', fontsize=14)
plt.ylabel('Task Difficulty', fontsize=14)
# plt.title('Average Task Difficulty: Kinesthetic Teaching vs Teleoperation')
plt.xticks(index + bar_width / 2, ['Pick and Place', 'Object Stacking', 'Object Insertion', 'Overall'])
plt.legend()

plt.show()
