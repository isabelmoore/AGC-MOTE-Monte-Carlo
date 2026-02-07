import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re
import matplotlib.ticker as ticker
from matplotlib import cm, colors

from datetime import datetime

def plot_yaw_correction_data_3d(file_path):
    # Initialize lists for storing data
    yaw_corrections = []
    costs = []
    true_yaws = []
    true_costs = []
    best_yaws = []
    best_costs = []
    timestamps = []
    true_timestamps = []


    # Read the file manually to separate regular and special entries
    with open(file_path, 'r') as file:
        next(file)  # Skip the header
        for line in file:
            if 'True Yaw' in line:
                # Handle the special summary entry
                parts = line.split(',')
                timestamp = parts[0].split(': ')[1].strip()
                true_yaw = float(parts[1].split(': ')[1].strip())
                true_cost = float(parts[2].split(': ')[1].strip())
                best_yaw = float(parts[3].split(': ')[1].strip())
                best_cost = float(parts[4].split(': ')[1].strip())
                true_timestamps.append(datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S"))
                true_yaws.append(true_yaw)
                true_costs.append(true_cost)
                best_yaws.append(best_yaw)
                best_costs.append(best_cost)
            else:
                # Handle regular data entry
                parts = line.split(',')
                timestamp = parts[0].strip()
                yaw_correction = float(parts[1].strip())
                cost = float(parts[2].strip())
                timestamps.append(datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S"))
                yaw_corrections.append(yaw_correction)
                costs.append(cost)

    # Convert lists to numpy arrays for plotting
    timestamps = np.array(timestamps)
    yaw_corrections = np.array(yaw_corrections)
    costs = np.array(costs)
    true_timestamps = np.array(true_timestamps)
    true_yaws = np.array(true_yaws)
    true_costs = np.array(true_costs)
    best_yaws = np.array(best_yaws)
    best_costs = np.array(best_costs)

    # Plotting the data
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Surface plot for regular entries
    ax.plot_trisurf(yaw_corrections, (timestamps - timestamps.min()).astype('timedelta64[s]').astype(int), costs, cmap='viridis', alpha=0.6)

    # Scatter plot for special entries
    ax.scatter(true_yaws, (true_timestamps - timestamps.min()).astype('timedelta64[s]').astype(int), true_costs, color='purple', label='True Yaw Costs', s=6)
    ax.scatter(best_yaws, (true_timestamps - timestamps.min()).astype('timedelta64[s]').astype(int), best_costs, color='b', label='Best Yaw Costs', s=6)

    ax.set_xlabel('Yaw Correction (radians)')
    ax.set_ylabel('Time (seconds from start)')
    ax.set_zlabel('Cost')
    ax.set_title('Monte Carlo Dead Reckoning for Yaw Correction')
    ax.legend()
    # ax.set_zlim(0, 0.2)
    plt.show()
# Example call to the function
file_path = 'monte_carlo_results.txt'
# plot_yaw_correction_data_3d(file_path)


def parse_log_file(file_path):
    data = []
    try:
        with open(file_path, 'r') as file:
            file.readline()  # Skip header
            for line in file:
                parts = line.split(", ", 6)  # Split into timestamp, q_multiplier, and the rest
                if len(parts) == 7:
                    timestamp = datetime.strptime(parts[0], "%Y-%m-%d %H:%M:%S")
                    q_multiplier = float(parts[1])
                    
                    # Extract R values
                    r_pos = float(parts[2].split(":")[1])
                    r_vel = float(parts[3].split(":")[1])
                    r_yaw = float(parts[4].split(":")[1])
                    r_yawrate = float(parts[5].split(":")[1])
                    
                    # State vector is already in the last part
                    state_values = [float(num) for num in re.findall(r"[-\d\.]+e?[-+]?\d*", parts[6])]

                    row = [timestamp, q_multiplier, r_pos, r_vel, r_yaw, r_yawrate] + state_values
                    data.append(row)

        columns = ['timestamp', 'q_multiplier', 'r_pos', 'r_vel', 'r_yaw', 'r_yawrate', 'x', 'y', 'velocity', 'yaw', 'yaw_rate']
        df = pd.DataFrame(data, columns=columns)
        if df.empty:
            print("No valid data after parsing.")
        return df
    except Exception as e:
        print(f"Error during parsing: {e}")
        return pd.DataFrame()  # Return an empty DataFrame on error


def plot_kalman_filter_results(file_path):
    df = parse_log_file(file_path)
    if df is not None and not df.empty:
        df['timestamp'] = df['timestamp'].apply(lambda x: x.timestamp())  # Convert datetime to timestamp
        df['yaw'] = df['yaw'] * (np.pi / 180)

        # Colors for different R measurements
        colors = ['red', 'orange', 'green', 'blue']
        state_variables = ['x', 'y', 'velocity', 'yaw', 'yaw_rate']  # Correct state variable names
        labels = ['X Position', 'Y Position', 'Velocity', 'Yaw', 'Yaw Rate']
        r_values = ['r_pos', 'r_vel', 'r_yaw', 'r_yawrate']
        
        fig = plt.figure(figsize=(15, 10))

        # Create subplots for each state variable
        for i, (state_var, label) in enumerate(zip(state_variables, labels)):
            ax = fig.add_subplot(2, 3, i + 1, projection='3d')
            for j, (r_value, color) in enumerate(zip(r_values, colors)):
                ax.scatter(df['timestamp'], df[r_value], df[state_var], color=color, label=f'{r_values[j]} ({label})')
            ax.set_xlabel('Timestamp')
            ax.set_ylabel('R value')
            ax.set_zlabel(label)
            ax.legend()

        plt.tight_layout()
        plt.show()
    else:
        print("No valid data to plot or DataFrame is empty.")


file_path = "/home/wizard/sharf/kalman_filter_results.txt"
plot_kalman_filter_results(file_path)
