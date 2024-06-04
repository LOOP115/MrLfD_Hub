import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the data
file_path = '../tasks_data.csv'
tasks_data = pd.read_csv(file_path)

# Extract the unique tasks
tasks = tasks_data['Task ID'].unique()

# Calculate mean and standard deviation for each task and method
summary = tasks_data.groupby(['Task ID', 'Method']).agg({'Attempts': ['mean', 'std']}).reset_index()
summary.columns = ['Task ID', 'Method', 'Mean', 'Std']
# Prepare data for plotting
kinesthetic_means = summary[summary['Method'] == 'k']['Mean'].values
teleoperation_means = summary[summary['Method'] == 't']['Mean'].values
kinesthetic_stds = summary[summary['Method'] == 'k']['Std'].values
teleoperation_stds = summary[summary['Method'] == 't']['Std'].values

# Define the p-values
p_values = {
    1: 0.919606,
    2: 0.162791,
    3: 0.395328
}

# Plotting
fig, ax = plt.subplots()

bar_width = 0.35
index = np.arange(len(tasks))

# Bar plots
bars1 = ax.bar(index, kinesthetic_means, bar_width, yerr=kinesthetic_stds, label='Kinesthetic Teaching', capsize=5)
bars2 = ax.bar(index + bar_width, teleoperation_means, bar_width, yerr=teleoperation_stds, label='Teleoperation', capsize=5)

# Labels and title
ax.set_xlabel('Task')
ax.set_ylabel('Mean Attempts')
ax.set_title('Mean Attempts by Task and Method')
ax.set_xticks(index + bar_width / 2)
ax.set_xticklabels(tasks)
ax.legend()

# Annotate mean and standard deviation on top of each bar
for i, bar in enumerate(bars1):
    height = bar.get_height()
    std_dev = kinesthetic_stds[i]
    ax.annotate(f'{height:.2f}\n±{std_dev:.2f}',
                xy=(bar.get_x() + bar.get_width() / 2, 0),
                xytext=(0, 3),  # 3 points vertical offset
                textcoords="offset points",
                ha='center', va='bottom')

for i, bar in enumerate(bars2):
    height = bar.get_height()
    std_dev = teleoperation_stds[i]
    ax.annotate(f'{height:.2f}\n±{std_dev:.2f}',
                xy=(bar.get_x() + bar.get_width() / 2, 0),
                xytext=(0, 3),  # 3 points vertical offset
                textcoords="offset points",
                ha='center', va='bottom')


ax.annotate(f'p = {p_values[1]:.6f} > 0.05',
                xy=(index[0] + bar_width / 2, 1.35),
                xytext=(0, 10),  # 10 points vertical offset
                textcoords="offset points",
                ha='center', va='bottom')

ax.annotate(f'p = {p_values[2]:.6f} > 0.05',
                xy=(index[1] + bar_width / 2, 1.8),
                xytext=(0, 10),  # 10 points vertical offset
                textcoords="offset points",
                ha='center', va='bottom')

ax.annotate(f'p = {p_values[3]:.6f} > 0.05',
                xy=(index[2] + bar_width / 2, 2.55),
                xytext=(0, 10),  # 10 points vertical offset
                textcoords="offset points",
                ha='center', va='bottom')

tasks = ['Pick and Place', 'Object Stacking', 'Object Insertion']
ax.set_xticklabels(tasks)
plt.tight_layout()
plt.show()
