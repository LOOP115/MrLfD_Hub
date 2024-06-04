import pandas as pd
import statsmodels.api as sm
from statsmodels.stats.anova import AnovaRM
from statsmodels.stats.multicomp import pairwise_tukeyhsd
import matplotlib.pyplot as plt
import seaborn as sns

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

# Reshape the data for repeated measures ANOVA
data_long = pd.melt(data, id_vars=['Participant', 'Task'], value_vars=[
    'Mental Demand - Kinesthetic', 'Physical Demand - Kinesthetic', 'Temporal Demand - Kinesthetic',
    'Performance - Kinesthetic', 'Effort - Kinesthetic', 'Frustration Level - Kinesthetic',
    'Mental Demand - Teleoperation', 'Physical Demand - Teleoperation', 'Temporal Demand - Teleoperation',
    'Performance - Teleoperation', 'Effort - Teleoperation', 'Frustration Level - Teleoperation'],
    var_name='Condition', value_name='Score')

# Split the Condition column into Dimension and Method
data_long[['Dimension', 'Method']] = data_long['Condition'].str.split(' - ', expand=True)
data_long = data_long.drop(columns=['Condition'])

# Perform repeated measures ANOVA
aovrm = AnovaRM(data_long, 'Score', 'Participant', within=['Dimension', 'Method'], aggregate_func='mean')
res = aovrm.fit()
print(res)

# Extract the ANOVA table
anova_table = res.anova_table

# Display the ANOVA table structure
print(anova_table)


# Perform Tukey's HSD post-hoc test for Dimension
tukey = pairwise_tukeyhsd(endog=data_long['Score'], groups=data_long['Dimension'], alpha=0.05)
print(tukey)

# Perform Tukey's HSD post-hoc test for Dimension:Method interaction
data_long['Interaction'] = data_long['Dimension'] + ':' + data_long['Method']
tukey_interaction = pairwise_tukeyhsd(endog=data_long['Score'], groups=data_long['Interaction'], alpha=0.05)
print(tukey_interaction)


# Interaction plot
plt.figure(figsize=(12, 6))
sns.pointplot(data=data_long, x='Dimension', y='Score', hue='Method', markers=['o', 's'], linestyles=['-', '--'])
plt.title('Interaction Plot: Method and NASA-TLX Dimensions')
plt.xlabel('NASA-TLX Dimension')
plt.ylabel('Score')
plt.legend(title='Method')
plt.show()
