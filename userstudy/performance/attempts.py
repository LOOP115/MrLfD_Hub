# Correct the calculation to include the overall average across both methods
import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the uploaded file
file_path = '../tasks_data.csv'
data = pd.read_csv(file_path)

# Calculate the mean and standard deviation of attempts for each task and method
task_methods = data.groupby(['Task ID', 'Method'])['Attempts'].agg(['mean', 'std']).reset_index()

# Calculate the average of both methods for each task
task_avg = data.groupby('Task ID')['Attempts'].agg(['mean', 'std']).reset_index()
task_avg['Method'] = 'avg'

# Combine the data
combined_data = pd.concat([task_methods, task_avg])

# Add a row for 'All Tasks' by combining all task data
all_tasks_mean_std = data.groupby('Method')['Attempts'].agg(['mean', 'std']).reset_index()
all_tasks_mean_std['Task ID'] = 'All Tasks'

# Calculate the overall average across all methods and tasks
overall_avg_mean = data['Attempts'].mean()
overall_avg_std = data['Attempts'].std()

# Add the overall average to the combined data
all_tasks_avg = pd.DataFrame({
    'Task ID': ['All Tasks'],
    'Method': ['avg'],
    'mean': [overall_avg_mean],
    'std': [overall_avg_std]
})

# Append 'All Tasks' data to the existing combined data
combined_data_with_all = pd.concat([task_methods, task_avg, all_tasks_mean_std, all_tasks_avg], ignore_index=True)

# Update the plot data pivot table
plot_data_with_all = combined_data_with_all.pivot(index='Task ID', columns='Method', values=['mean', 'std'])

# Plotting with the corrected data
fig, ax = plt.subplots(figsize=(14, 4))

methods = ['k', 't', 'avg']
tasks_with_all = ['Pick and Place', 'Object Stacking', 'Object Insertion', 'All Tasks']
bar_width = 0.2
index = range(len(tasks_with_all))

# Plot each method
for i, method in enumerate(methods):
    means = plot_data_with_all['mean'][method]
    stds = plot_data_with_all['std'][method]
    bar_positions = [x + i*bar_width for x in index]
    label = 'Kinesthetic Teaching' if method == 'k' else 'Teleoperation' if method == 't' else 'Overall'
    bars = ax.bar(bar_positions, means, bar_width, yerr=stds, capsize=5, label=label)
    # Label the mean and standard deviation on each bar
    for bar, mean, std in zip(bars, means, stds):
        yval = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2.0, yval - bar.get_height() + 0.1, f'{mean:.2f}\n±{std:.2f}', ha='center', va='bottom', fontsize=11)

# Customize the plot
ax.set_xlabel('Tasks', fontsize=14)
ax.set_ylabel('Average Number of Attempts', fontsize=14)
ax.set_title('Average Number of Attempts by Method and Task', fontsize=16)
ax.set_xticks([r + bar_width for r in index])
ax.set_xticklabels(tasks_with_all)
ax.legend()

plt.grid(True)
plt.show()
