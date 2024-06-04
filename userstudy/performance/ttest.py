import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats

# Load the data
tasks_data = pd.read_csv('../tasks_data.csv')


# Function to perform repeated t-test for each task
def perform_repeated_t_test(data, task_id):
    # Filter data for the given task
    task_data = data[data['Task ID'] == task_id]

    # Separate attempts by method
    k_attempts = task_data[task_data['Method'] == 'k']['Attempts']
    t_attempts = task_data[task_data['Method'] == 't']['Attempts']

    # Perform the repeated t-test
    t_stat, p_value = stats.ttest_rel(k_attempts, t_attempts, alternative="less")

    return t_stat, p_value


# Perform the t-test for each task
results = {}
for task_id in tasks_data['Task ID'].unique():
    t_stat, p_value = perform_repeated_t_test(tasks_data, task_id)
    results[task_id] = {'t-statistic': t_stat, 'p-value': p_value}

# Display the results
results_df = pd.DataFrame(results).T
print(results_df)
