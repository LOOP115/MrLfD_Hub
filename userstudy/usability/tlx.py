import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the data from the uploaded file
file_path = '../tasks-nc.csv'
data = pd.read_csv(file_path)

# Rename columns for easier reference
data = data.rename(columns={
    'NASA-TLX-Kinesthetic teaching - How mentally demanding was the task?': 'Mental Demand - Kinesthetic',
    'NASA-TLX-Kinesthetic teaching - How physically demanding was the task?': 'Physical Demand - Kinesthetic',
    'NASA-TLX-Kinesthetic teaching - How hurried or rushed was the pace of the task?': 'Temporal Demand - Kinesthetic',
    'NASA-TLX-Kinesthetic teaching - How successful were you in accomplishing what you were asked to do?': 'Performance - Kinesthetic',
    'NASA-TLX-Kinesthetic teaching - How hard did you have to work to accomplish your level of performance?': 'Effort - Kinesthetic',
    'NASA-TLX-Kinesthetic teaching - How insecure, discouraged, irritated, stressed and annoyed were you?': 'Frustration Level - Kinesthetic',
    'NASA-TLX-Teleoperation - How mentally demanding was the task?': 'Mental Demand - Teleoperation',
    'NASA-TLX-Teleoperation - How physically demanding was the task?': 'Physical Demand - Teleoperation',
    'NASA-TLX-Teleoperation - How hurried or rushed was the pace of the task?': 'Temporal Demand - Teleoperation',
    'NASA-TLX-Teleoperation - How successful were you in accomplishing what you were asked to do?': 'Performance - Teleoperation',
    'NASA-TLX-Teleoperation - How hard did you have to work to accomplish your level of performance?': 'Effort - Teleoperation',
    'NASA-TLX-Teleoperation - How insecure, discouraged, irritated, stressed and annoyed were you?': 'Frustration Level - Teleoperation'
})

# Calculate the Task Load Index
data['Task Load Index - Kinesthetic'] = data[
    ['Mental Demand - Kinesthetic', 'Physical Demand - Kinesthetic', 'Temporal Demand - Kinesthetic', 'Performance - Kinesthetic', 'Effort - Kinesthetic', 'Frustration Level - Kinesthetic']
].mean(axis=1)

data['Task Load Index - Teleoperation'] = data[
    ['Mental Demand - Teleoperation', 'Physical Demand - Teleoperation', 'Temporal Demand - Teleoperation', 'Performance - Teleoperation', 'Effort - Teleoperation', 'Frustration Level - Teleoperation']
].mean(axis=1)

# Prepare data for plotting
nasa_tlx_df = pd.DataFrame({
    'Component': ['Mental Demand', 'Physical Demand', 'Temporal Demand', 'Performance', 'Effort', 'Frustration Level', 'Task Load Index'],
    'Kinesthetic Teaching': [
        data['Mental Demand - Kinesthetic'].mean(),
        data['Physical Demand - Kinesthetic'].mean(),
        data['Temporal Demand - Kinesthetic'].mean(),
        data['Performance - Kinesthetic'].mean(),
        data['Effort - Kinesthetic'].mean(),
        data['Frustration Level - Kinesthetic'].mean(),
        data['Task Load Index - Kinesthetic'].mean()
    ],
    'Teleoperation': [
        data['Mental Demand - Teleoperation'].mean(),
        data['Physical Demand - Teleoperation'].mean(),
        data['Temporal Demand - Teleoperation'].mean(),
        data['Performance - Teleoperation'].mean(),
        data['Effort - Teleoperation'].mean(),
        data['Frustration Level - Teleoperation'].mean(),
        data['Task Load Index - Teleoperation'].mean()
    ]
})

# Calculating standard deviation for the NASA-TLX components
nasa_tlx_std = pd.DataFrame({
    'Kinesthetic Teaching': [
        data['Mental Demand - Kinesthetic'].std(),
        data['Physical Demand - Kinesthetic'].std(),
        data['Temporal Demand - Kinesthetic'].std(),
        data['Performance - Kinesthetic'].std(),
        data['Effort - Kinesthetic'].std(),
        data['Frustration Level - Kinesthetic'].std(),
        data['Task Load Index - Kinesthetic'].std()
    ],
    'Teleoperation': [
        data['Mental Demand - Teleoperation'].std(),
        data['Physical Demand - Teleoperation'].std(),
        data['Temporal Demand - Teleoperation'].std(),
        data['Performance - Teleoperation'].std(),
        data['Effort - Teleoperation'].std(),
        data['Frustration Level - Teleoperation'].std(),
        data['Task Load Index - Teleoperation'].std()
    ]
})

# Plotting the bar chart without the overall bars
plt.figure(figsize=(20, 4))
bar_width = 0.4
index = np.arange(len(nasa_tlx_df))

# Plot bars for kinesthetic teaching with error bars
plt.bar(index, nasa_tlx_df['Kinesthetic Teaching'], bar_width, yerr=nasa_tlx_std['Kinesthetic Teaching'], capsize=5, label='Kinesthetic Teaching')

# Plot bars for teleoperation with error bars
plt.bar(index + bar_width, nasa_tlx_df['Teleoperation'], bar_width, yerr=nasa_tlx_std['Teleoperation'], capsize=5, label='Teleoperation')

# Add labels for mean and standard deviation values
for i in range(len(nasa_tlx_df)):
    plt.text(index[i], nasa_tlx_df['Kinesthetic Teaching'][i], f'Mean: {nasa_tlx_df["Kinesthetic Teaching"][i]:.2f}\nStd: {nasa_tlx_std["Kinesthetic Teaching"][i]:.2f}', horizontalalignment='center', verticalalignment='bottom', color='blue', fontsize=10)
    plt.text(index[i] + bar_width, nasa_tlx_df['Teleoperation'][i], f'Mean: {nasa_tlx_df["Teleoperation"][i]:.2f}\nStd: {nasa_tlx_std["Teleoperation"][i]:.2f}', horizontalalignment='center', verticalalignment='bottom', color='red', fontsize=10)

# Add labels, title, and legend
# plt.xlabel('NASA-TLX')
plt.ylabel('Score')
# plt.title('NASA-TLX: Kinesthetic Teaching vs Teleoperation')
plt.xticks(index + bar_width / 2, nasa_tlx_df['Component'])
plt.legend()
plt.yticks(np.arange(0, 21, 5))

plt.show()
