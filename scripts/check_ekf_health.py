import numpy as np
import os

path = 'results/kalman_filter_results.txt'
if os.path.exists(path):
    # Use genfromtxt to handle messy headers or missing data
    data = np.genfromtxt(path, delimiter=',', skip_header=1)
    
    if data.ndim == 1:
        print("Not enough data.")
    else:
        # Last 100 points
        subset = data[-100:]
        
        # Columns: Time, N, E, V, Yaw, Bias, Yaw_Raw, Yaw_Rate
        v_mean = np.mean(subset[:, 3])
        yaw_mean = np.mean(subset[:, 4])
        bias_mean = np.mean(subset[:, 5])
        raw_mean = np.mean(subset[:, 6])
        
        # Check for NaNs
        nans = np.isnan(data).any()
        
        print(f"--- EKF Health Check (Last 100 Samples) ---")
        print(f"NaNs detected:  {nans}")
        print(f"Mean Velocity:  {v_mean:.4f} m/s")
        print(f"Mean EKF Yaw:   {yaw_mean:.4f} rad")
        print(f"Mean Raw Yaw:   {raw_mean:.4f} rad")
        print(f"Mean Bias:      {bias_mean:.4f} rad/s")
        print(f"Yaw Diff (rad): {yaw_mean - raw_mean:.4f}")
        print(f"Yaw Diff (deg): {np.degrees(yaw_mean - raw_mean):.2f}")
        
        # Check for signs of reverse
        print(f"Min Velocity:   {np.min(subset[:, 3]):.4f}")
        print(f"Max Velocity:   {np.max(subset[:, 3]):.4f}")
        
        if np.any(subset[:, 3] < -0.1):
            print("ALERT: Negative velocity detected! Vehicle is in REVERSE.")
        
        # Check for bias runaway
        if abs(bias_mean) > 1.0:
            print("ALERT: Bias is excessive! (>1 rad/s)")
else:
    print("Results file not found.")
