import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the uploaded file
file_path = '../tasks-nc.csv'
data = pd.read_csv(file_path)

# Rename columns for easier reference
data = data.rename(columns={
    'UX - I felt like I was controlling the movements of the robotic arm during teleoperation': 'Control',
    'UX - The movements of the robotic arm were in sync with my own movements': 'Sync'
})

# Calculate participants' satisfactory on teleoperation movement
data['Satisfactory'] = data[['Control', 'Sync']].mean(axis=1)

# Calculate the mean and standard deviation for each task and the overall mean and standard deviation
task_means = data.groupby('Task')['Satisfactory'].mean()
task_stds = data.groupby('Task')['Satisfactory'].std()
overall_mean = data['Satisfactory'].mean()
overall_std = data['Satisfactory'].std()

# Custom x labels
x_labels = ['Pick and Place', 'Object Stacking', 'Object Insertion', 'Overall']

# Create a plot with custom x labels and standard deviation labels under the mean
plt.figure(figsize=(8, 4))
bars = plt.bar(x_labels, task_means.tolist() + [overall_mean],
               yerr=task_stds.tolist() + [overall_std], color=['blue', 'blue', 'blue', 'green'], capsize=5)

# Adding labels and title
plt.xlabel('Task')
plt.ylabel('Mean Satisfactory Score')
# plt.title('Mean Satisfactory Score on Teleoperation Movement by Task')
plt.ylim(0, 10)

# Adding the value labels on top of the bars
for bar, std in zip(bars, task_stds.tolist() + [overall_std]):
    yval = bar.get_height()
    plt.text(bar.get_x() + 0.7, yval + 0.1, f'Mean: {round(yval, 2)}\nStd: {round(std, 2)})', ha='center')

# Show the plot
plt.show()
