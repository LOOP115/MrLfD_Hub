import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the uploaded file
file_path = '../tasks-nc.csv'
data = pd.read_csv(file_path)

# Rename columns for easier reference
data = data.rename(columns={
    'UX - The visualisations help me better control the robotic arm during kinesthetic teaching': 'Visualisation_Kinesthetic',
    'UX - The visualisations help me better control the robotic arm during teleoperation': 'Visualisation_Teleoperation'
})

# Calculate the mean and standard deviation for each task and the overall mean and standard deviation
kinesthetic_means = data.groupby('Task')['Visualisation_Kinesthetic'].mean()
teleoperation_means = data.groupby('Task')['Visualisation_Teleoperation'].mean()
kinesthetic_stds = data.groupby('Task')['Visualisation_Kinesthetic'].std()
teleoperation_stds = data.groupby('Task')['Visualisation_Teleoperation'].std()
overall_kinesthetic_mean = data['Visualisation_Kinesthetic'].mean()
overall_teleoperation_mean = data['Visualisation_Teleoperation'].mean()
overall_kinesthetic_std = data['Visualisation_Kinesthetic'].std()
overall_teleoperation_std = data['Visualisation_Teleoperation'].std()

# Create a plot with custom x labels and two bars for each group including standard deviation
x_labels = ['Pick and Place', 'Object Stacking', 'Object Insertion', 'Overall']
x = range(len(x_labels))

plt.figure(figsize=(12, 6))

# Create the bars
width = 0.35
bars1 = plt.bar([i - width/2 for i in x], kinesthetic_means.tolist() + [overall_kinesthetic_mean],
                yerr=kinesthetic_stds.tolist() + [overall_kinesthetic_std], width=width, label='Kinesthetic Teaching', color='blue', capsize=5)
bars2 = plt.bar([i + width/2 for i in x], teleoperation_means.tolist() + [overall_teleoperation_mean],
                yerr=teleoperation_stds.tolist() + [overall_teleoperation_std], width=width, label='Teleoperation', color='orange', capsize=5)

# Adding labels and title
plt.xlabel('Task')
plt.ylabel('Mean Score')
# plt.title('Mean Score on Visualisation Helpfulness by Task')
plt.xticks(ticks=x, labels=x_labels)
plt.ylim(0, 10)

# Adding the value labels on top of the bars
for bars, stds in zip([bars1, bars2], [kinesthetic_stds.tolist() + [overall_kinesthetic_std], teleoperation_stds.tolist() + [overall_teleoperation_std]]):
    for bar, std in zip(bars, stds):
        yval = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2, 1, f'Mean: {round(yval, 2)}\nStd:{round(std, 2)}', ha='center')

plt.legend()

# Show the plot
plt.show()
