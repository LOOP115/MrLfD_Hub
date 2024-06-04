import pandas as pd
import matplotlib.pyplot as plt

# Load the demographics data from the uploaded CSV file
file_path = '../demographics.csv'
demographics_df = pd.read_csv(file_path)


# Calculate mean and standard deviation for each activity
activities = [
    'Background - How often do you play videogames?',
    'Background - How often do you use mixed reality applications (AR/VR)?',
    'Background - How often do you interact with robotic arms?',
    'Background - How often do you engage in activities requiring upper limb coordination?'
]

# Correct column selection for background questions
background_data_corrected = demographics_df.iloc[:, 3:7]

# Calculate the mean and standard deviation for each question
means_corrected = background_data_corrected.mean()
std_devs_corrected = background_data_corrected.std()

# Define short phrases for the four background questions correctly
background_questions_corrected = [
    "Videogame",
    "AR/VR App Usage",
    "Robotic Arm Interaction",
    "Upper Limb Activities"
]

# Plot the mean values with error bars representing standard deviation
plt.figure(figsize=(12, 8))
bars = plt.bar(background_questions_corrected, means_corrected, yerr=std_devs_corrected, capsize=10, color='skyblue', edgecolor='black')

# Label the mean and standard deviation on each bar
for bar, mean, std in zip(bars, means_corrected, std_devs_corrected):
    yval = bar.get_height()
    plt.text(bar.get_x() + 0.65, yval + 0.3, f'Mean: {mean:.2f}\nSD: {std:.2f}', ha='center', va='bottom', fontsize=13)

plt.xlabel('Background', fontsize=14)
plt.ylabel('Mean', fontsize=14)
plt.grid(True)
plt.show()
