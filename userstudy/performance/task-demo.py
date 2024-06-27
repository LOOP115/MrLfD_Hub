import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load the data from the uploaded files
tasks_file_path = '../tasks_data.csv'
demographics_file_path = '../demographics.csv'

tasks_data = pd.read_csv(tasks_file_path)
demographics_data = pd.read_csv(demographics_file_path)

# Calculate the average attempts and completion time for each participant across different methods
participant_stats = tasks_data.groupby(['Participant ID', 'Method']).agg({
    'Attempts': 'mean',
    'Time': 'mean'
}).reset_index()

# Pivot the table to have separate columns for kinesthetic teaching and teleoperation
participant_pivot = participant_stats.pivot(index='Participant ID', columns='Method', values=['Attempts', 'Time']).reset_index()

# Flatten the MultiIndex columns
participant_pivot.columns = [' '.join(col).strip() for col in participant_pivot.columns.values]

# Calculate the overall average for each participant
participant_pivot['Attempts_avg'] = participant_pivot[['Attempts k', 'Attempts t']].mean(axis=1)
participant_pivot['Time_avg'] = participant_pivot[['Time k', 'Time t']].mean(axis=1)

# Merge this table with the demographics data
merged_stats = pd.merge(participant_pivot, demographics_data, on='Participant ID')

# Select the first 10 participants for display
stats_table = merged_stats.head(10)

# Correlation analysis with the 4 background questions
background_questions = [
    "Background - How often do you play videogames?",
    "Background - How often do you use mixed reality applications (AR/VR)?",
    "Background - How often do you interact with robotic arms?",
    "Background - How often do you engage in activities requiring upper limb coordination?"
]

correlation_stats = stats_table[['Attempts_avg', 'Time_avg',
                                 'Attempts k', 'Time k',
                                 'Attempts t', 'Time t'] + background_questions]

corr_attempts_avg = correlation_stats.corr()['Attempts_avg'][background_questions]
corr_time_avg = correlation_stats.corr()['Time_avg'][background_questions]

corr_attempts_k = correlation_stats.corr()['Attempts k'][background_questions]
corr_time_k = correlation_stats.corr()['Time k'][background_questions]

corr_attempts_t = correlation_stats.corr()['Attempts t'][background_questions]
corr_time_t = correlation_stats.corr()['Time t'][background_questions]

# Combine the correlation results into a single dataframe for display
corr_stats_df = pd.DataFrame({
    'Average Attempts': corr_attempts_avg,
    'Average Time': corr_time_avg,
    'Kinesthetic Teaching Attempts': corr_attempts_k,
    'Kinesthetic Teaching Time': corr_time_k,
    'Teleoperation Attempts': corr_attempts_t,
    'Teleoperation Time': corr_time_t
})

# Combine all the correlation data into a single dataframe for heatmap
combined_corr_data = pd.DataFrame({
    'Average Attempts': corr_attempts_avg,
    'Average Time': corr_time_avg,
    'Kinesthetic Teaching Attempts': corr_attempts_k,
    'Kinesthetic Teaching Time': corr_time_k,
    'Teleoperation Attempts': corr_attempts_t,
    'Teleoperation Time': corr_time_t
}).T

# Shorten the background question phrases
short_background_questions = [
    "Videogames",
    "AR/VR Apps",
    "Robotic Arms",
    "Upper Limb"
]

combined_corr_data.columns = short_background_questions

# Plotting the heatmap with short phrases
plt.figure(figsize=(8, 6))  # Smaller and more compact plot size
sns.heatmap(combined_corr_data, annot=True, cmap='coolwarm', center=0, linewidths=.5)
plt.title('Correlation Heatmap with Background Questions', fontsize=16)
plt.xlabel('Background Questions', fontsize=12)
plt.ylabel('Metrics', fontsize=12)
plt.show()
