import pandas as pd
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib
plt.rcParams["figure.autolayout"] = True
#columns = ["Name", "Marks"]
df = pd.read_csv("logData/sensor_stabilizer_kalman_tasks.csv")

def df_drop_outliers(df, cols):
    # Remove rows where there is an outlier in any of the given columns
    
    
    outlier_indices = []

    for i in range(1, len(df) - 1):
        addi = False 
        for col in cols:
            current_value = df.at[i, col]
            prev_value = df.at[i - 1, col]
            next_value = df.at[i + 1, col]
            
            if current_value != prev_value + 1 and current_value != next_value - 1:
                # there is an outlier at index i
                addi = True
                break
        if(addi):
            outlier_indices.append(i)


    # Filter out the outlier rows
    return df.drop(outlier_indices)
print("cleaning data...")
df = df_drop_outliers(df, ["timer.stabloop"])
df["Timestamp"] -= df["Timestamp"].min()

print("plotting data...")
data = df.iloc[:20]
# Define task start and end columns
task_columns = {
    'SensorsTask': {'start': 'timer.sensorsstart', 'end': 'timer.sensorsend'},
    'StabilizerTask': {'start': 'timer.stabilizerstart', 'end': 'timer.stabilizerend'},
    'KalmanTask': {'start': 'timer.kalmanstart', 'end': 'timer.kalmanend'}
}

task_width = 0.6

# Find the global initial time point
min_time = min(data.min()[col] for task in task_columns.values() for col in task.values())
max_time = max(data.max()[col] for task in task_columns.values() for col in task.values())



# Function to plot rectangles representing tasks
def plot_tasks(ax, task_name, start_col, end_col, y_level):
    for index, row in df.iterrows():
        start = row[start_col]
        end = row[end_col]
        ax.add_patch(Rectangle((start, y_level - task_width/2), end - start, task_width))

# Create figure and axes
fig, ax = plt.subplots(figsize=(10, 6))


# Plot tasks
for task_name, columns in task_columns.items():
    plot_tasks(ax, task_name, columns['start'], columns['end'], list(task_columns.keys()).index(task_name) + 1)

if False:
    # Plot markers for estimatorcall
    estimator_calls = df['timer.estimatorcall']
    ax.scatter(estimator_calls, [1.5] * len(estimator_calls), color='red', label='Estimator Call')

# Set labels and legend
ax.set_xlabel('Time (microseconds)')
ax.set_ylabel('Tasks')
ax.set_title('Task Timeline')
ax.legend()

# Show plot
plt.grid(True)
plt.ylim(0.5, len(task_columns) + 0.5)
plt.yticks(range(1, len(task_columns)+1), list(task_columns.keys()))
plt.xlim(min_time, max_time)
plt.show()