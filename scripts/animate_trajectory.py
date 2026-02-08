
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import sys

# Path to results
RESULTS_FILE = os.path.abspath(os.path.join(os.path.dirname(__file__), '../results/kalman_filter_results.txt'))
SAVE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '../results/trajectory_animation.gif'))

def load_results(filepath):
    print(f"Loading results from {filepath}...", flush=True)
    if not os.path.exists(filepath):
        print("File does not exist.", flush=True)
        return np.array([])
    print("DEBUG: Using ROBUST load_results", flush=True)
    data = []
    expected_cols = None
    with open(filepath, 'r') as f:
        lines = f.readlines()
        # Skip header: Timestamp, N, E, V, Yaw, Bias
        for i, line in enumerate(lines[1:]):
            if not line.strip(): continue
            try:
                parts = [float(x.strip()) for x in line.split(',')]
                if expected_cols is None:
                    expected_cols = len(parts)
                
                if len(parts) == expected_cols:
                    data.append(parts)
                else:
                    # print(f"Skipping line {i+2}: Found {len(parts)} cols, expected {expected_cols}")
                    pass
            except ValueError:
                continue
    return np.array(data)

def animate_results():
    if not os.path.exists(RESULTS_FILE):
        print(f"Error: Results file not found at {RESULTS_FILE}")
        print("Please run the batch analysis first: docker-compose run mote_ros python3 scripts/analyze_bag_data.py")
        return

    import time
    max_retries = 30
    for i in range(max_retries):
        data = load_results(RESULTS_FILE)
        if len(data) > 0:
            break
        print(f"Waiting for data... ({i+1}/{max_retries})", flush=True)
        time.sleep(1)
        
    if len(data) == 0:
        print("No data found in results file after waiting.", flush=True)
        return
    
    print(f"Loaded {len(data)} points. Creating animation...")
    
    # Decimate for speed (plot every 500th point)
    decimate = 100
    data_dec = data[::decimate]
    
    # Setup Figure with 3 subplots (Map, Yaw, Velocity)
    fig = plt.figure(figsize=(10, 14))
    gs = fig.add_gridspec(3, 1, height_ratios=[3, 1.5, 1], hspace=0.4)
    
    ax_map = fig.add_subplot(gs[0])
    ax_yaw = fig.add_subplot(gs[1])
    ax_vel = fig.add_subplot(gs[2])
    
    # --- Map Plot Setup ---
    ax_map.set_aspect('equal')
    ax_map.grid(True)
    ax_map.set_xlabel('Relative Easting (m)')
    ax_map.set_ylabel('Relative Northing (m)')
    ax_map.set_title('Robot Trajectory with Corrected Yaw (EKF)')
    
    # Compass / Legend
    box_props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax_map.text(0.05, 0.95, "Global North (+Y)\nStandard Yaw 0 (+X)", transform=ax_map.transAxes, fontsize=10,
        verticalalignment='top', bbox=box_props)
    
    # Elements
    trail_line, = ax_map.plot([], [], 'r-', alpha=0.5, lw=1, label='Path')
    robot_marker, = ax_map.plot([], [], 'bo', ms=6, label='Robot')
    
    # Green = Corrected EKF Yaw
    arrow_line, = ax_map.plot([], [], 'g-', lw=2, label='Corrected Yaw (20m)')
    # Red = Raw INS Yaw
    arrow_raw_line, = ax_map.plot([], [], 'r-', lw=2, label='Raw INS Yaw (20m)')
    # Blue = Course Over Ground (Trajectory Truth)
    arrow_cog_line, = ax_map.plot([], [], 'b-', lw=2, label='Course Truth (20m)')
    
    # Set Limits (Map)
    min_e, max_e = np.min(data_dec[:, 1]), np.max(data_dec[:, 1]) # East is index 1
    min_n, max_n = np.min(data_dec[:, 2]), np.max(data_dec[:, 2]) # North is index 2
    margin = 50
    ax_map.set_xlim(min_e - margin, max_e + margin)
    ax_map.set_ylim(min_n - margin, max_n + margin)
    ax_map.legend(loc='upper right', fontsize='small')
    
    time_text = ax_map.text(0.02, 0.02, '', transform=ax_map.transAxes)
    
    # --- Yaw Plot Setup ---
    time_arr = data[:, 0]
    ekf_yaw_deg = np.degrees(data[:, 4])
    raw_yaw_deg = np.degrees(data[:, 6])
    
    # Helper for cleaning wraps
    def clean_for_plotting(t, y, threshold=50):
        # Calculate diffs
        dy = np.diff(y)
        # Find jumps
        jump_inds = np.where(np.abs(dy) > threshold)[0]
        
        if len(jump_inds) == 0:
            return t, y
            
        # Insert NaNs
        t_clean = np.insert(t, jump_inds+1, np.nan)
        y_clean = np.insert(y, jump_inds+1, np.nan)
        return t_clean, y_clean

    # Clean EKF
    ekf_t, ekf_y = clean_for_plotting(time_arr, ekf_yaw_deg)
    ax_yaw.plot(ekf_t, ekf_y, 'g-', lw=2, label='EKF Yaw')
    
    # Clean Raw INS
    raw_t, raw_y = clean_for_plotting(time_arr, raw_yaw_deg)
    ax_yaw.plot(raw_t, raw_y, 'r--', alpha=0.6, label='Raw INS Yaw')
    
    # Calculate Course Truth for plot
    # Use a longer stride to filter out GPS noise when moving slowly
    stride = 100 
    if len(data) > stride:
        t_sub = time_arr[::stride]
        x_sub = data[::stride, 1] # East
        y_sub = data[::stride, 2] # North
        
        dx = np.diff(x_sub)
        dy = np.diff(y_sub)
        dist = np.hypot(dx, dy)
        
        # Only show course if moved > 0.5m over the stride (approx 0.5m/s if 100Hz)
        valid_mask = dist > 0.5 
        
        if np.any(valid_mask):
            valid_t = t_sub[:-1][valid_mask]
            
            # Raw Course Angles
            cog_angles = np.arctan2(dy[valid_mask], dx[valid_mask])
            cog_deg = np.degrees(cog_angles)
            
            # Smooth Course Angles (Unwrap -> Smooth -> Wrap)
            cog_unwrapped = np.unwrap(cog_angles)
            window = 10
            cog_smooth_unwrapped = np.convolve(cog_unwrapped, np.ones(window)/window, mode='same')
            cog_smooth_rad = (cog_smooth_unwrapped + np.pi) % (2 * np.pi) - np.pi
            cog_smooth_deg = np.degrees(cog_smooth_rad)

            # --- Reverse Detection ---
            # Interpolate Raw Yaw to valid_t to compare
            # Subsampled raw yaw:
            raw_yaw_sub = data[::stride, 6][:-1][valid_mask] # Matches valid_t size
            
            # Difference between Course and Raw Yaw
            diff = np.abs(np.degrees(np.arctan2(np.sin(cog_smooth_rad - raw_yaw_sub), np.cos(cog_smooth_rad - raw_yaw_sub))))
            
            # Re-wrap
            cog_smooth_deg = (cog_smooth_deg + 180) % 360 - 180

            # Clean Course Plots
            cog_raw_t, cog_raw_y = clean_for_plotting(valid_t, cog_deg)
            cog_smooth_t, cog_smooth_y = clean_for_plotting(valid_t, cog_smooth_deg)

            # Plot faint original (Raw Course), bold smoothed (Reversed & Cleaned)
            ax_yaw.plot(cog_raw_t, cog_raw_y, 'b-', lw=1, alpha=0.3, label='Course Truth (Raw)')
            ax_yaw.plot(cog_smooth_t, cog_smooth_y, 'b-', lw=2, alpha=0.8, label='Course Truth (Smoothed)')

    ax_yaw.set_ylabel('Yaw (deg)')
    ax_yaw.set_title('Yaw Comparison: Corrected vs Raw vs Course')
    ax_yaw.set_yticks([-180, -90, 0, 90, 180])
    ax_yaw.set_yticklabels(['W (-180)', 'S (-90)', 'E (0)', 'N (90)', 'W (180)'])
    ax_yaw.set_ylim(-250, 250)
    ax_yaw.legend(loc='upper right', fontsize='small')
    ax_yaw.grid(True)
    
    # Moving vertical line for Yaw
    yaw_time_line = ax_yaw.axvline(data[0, 0], color='k', lw=1, alpha=0.5)

    # --- Velocity Plot Setup ---
    # Plot full velocity history as background
    ax_vel.plot(data[:, 0], data[:, 3], 'k-', alpha=0.3, lw=1, label='Velocity')
    ax_vel.axhline(0.3, color='m', linestyle='--', alpha=0.5, label='Fusion Threshold (0.3m/s)')
    
    # Moving marker for velocity
    vel_marker, = ax_vel.plot([], [], 'ro', ms=6, label='Current Speed')
    
    ax_vel.set_ylabel('Velocity (m/s)')
    ax_vel.set_xlabel('Time (s)')
    ax_vel.set_title('Velocity & Yaw Bias Profile')
    ax_vel.legend(loc='upper left', fontsize='small')
    ax_vel.grid(True)
    
    # --- Yaw Bias Plot (Twin Axis) ---
    ax_bias = ax_vel.twinx()
    # Plot full Bias history as background
    ax_bias.plot(data[:, 0], data[:, 5], 'tab:orange', alpha=0.6, lw=1.5, label='Yaw Bias (rad/s)')
    # Moving marker for Bias
    bias_marker, = ax_bias.plot([], [], 'o', color='tab:orange', ms=6, label='Current Bias')
    
    ax_bias.set_ylabel('Yaw Bias (rad/s)', color='tab:orange')
    ax_bias.tick_params(axis='y', labelcolor='tab:orange')
    ax_bias.legend(loc='upper right', fontsize='small')
    
    def update(i):
        # Easting (X), Northing (Y)
        ei = data_dec[i, 1] 
        ni = data_dec[i, 2] 
        yawi = data_dec[i, 4] # Corrected
        raw_yawi = data_dec[i, 6] # Raw
        ti = data_dec[i, 0]
        vi = data_dec[i, 3] # Velocity
        bi = data_dec[i, 5] # Bias
        
        # Trail
        trail_line.set_data(data_dec[:i+1, 1], data_dec[:i+1, 2])
        
        # Marker (Map)
        robot_marker.set_data([ei], [ni])
        
        # Marker (Velocity)
        vel_marker.set_data([ti], [vi])
        
        # Marker (Bias)
        bias_marker.set_data([ti], [bi])
        
        # Line (Yaw)
        yaw_time_line.set_xdata([ti, ti])
        
        # Heading Vectors (20m)
        len_vec = 20.0
        
        # 1. Corrected (Green)
        U = len_vec * np.cos(yawi)
        V = len_vec * np.sin(yawi)
        arrow_line.set_data([ei, ei+U], [ni, ni+V])
        
        # 2. Raw (Red)
        U_raw = len_vec * np.cos(raw_yawi)
        V_raw = len_vec * np.sin(raw_yawi)
        arrow_raw_line.set_data([ei, ei+U_raw], [ni, ni+V_raw])
        
        # 3. Course Over Ground (Blue) - "Truth" from movement
        if i > 0:
            # Look back 1 frame (or more if needed)
            prev_ei = data_dec[i-1, 2]
            prev_ni = data_dec[i-1, 1]
            dx = ei - prev_ei
            dy = ni - prev_ni
            
            # Only draw if moving
            dist = np.hypot(dx, dy)
            if dist > 0.1:
                cog_angle = np.arctan2(dy, dx)
                U_cog = len_vec * np.cos(cog_angle)
                V_cog = len_vec * np.sin(cog_angle)
                arrow_cog_line.set_data([ei, ei+U_cog], [ni, ni+V_cog])
            else:
                 arrow_cog_line.set_data([], [])
        else:
            arrow_cog_line.set_data([], [])
            
        time_text.set_text(f'Time: {ti:.1f}s | Yaw: {np.degrees(yawi):.1f} deg | Vel: {vi:.2f} m/s | Rate: {data_dec[i, 7]:.3f} rad/s')
        return trail_line, robot_marker, arrow_line, arrow_raw_line, arrow_cog_line, time_text, vel_marker, yaw_time_line, bias_marker


    ani = animation.FuncAnimation(fig, update, frames=len(data_dec), interval=30, blit=True)
    
    print(f"Rendering {len(data_dec)} frames... this may take a moment...", flush=True)
    print(f"Saving animation to {SAVE_PATH} ...", flush=True)
    ani.save(SAVE_PATH, writer='pillow', fps=30)
    print("Animation saved!", flush=True)

if __name__ == "__main__":
    animate_results()
