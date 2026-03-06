
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
    decimate = 300
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
    raw_yaw_deg = np.degrees(data[:, 8])
    
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
    # Convert Raw NED Yaw to ENU
    # Formula: ENU = (90 - NED + 180) % 360 - 180
    raw_yaw_enu = (90 - raw_yaw_deg + 180) % 360 - 180
    
    raw_t, raw_y = clean_for_plotting(time_arr, raw_yaw_enu)
    ax_yaw.plot(raw_t, raw_y, 'r--', alpha=0.6, label='Raw INS Yaw (ENU)')
    
    # --- Pre-calculate Smoothed Truth for Error Metrics ---
    stride_gt = 100
    cog_smooth_lookup = None # Will store (time, yaw)
    if len(data) > stride_gt:
        t_gt = time_arr[::stride_gt]
        x_gt = data[::stride_gt, 9] # Use Raw GPS East (Column 9)
        y_gt = data[::stride_gt, 10] # Use Raw GPS North (Column 10)
        dx_gt = np.diff(x_gt)
        dy_gt = np.diff(y_gt)
        dist_gt = np.hypot(dx_gt, dy_gt)
        valid_gt = dist_gt > 0.5
        
        if np.any(valid_gt):
            valid_t_gt = t_gt[:-1][valid_gt]
            cog_angles_gt = np.arctan2(dy_gt[valid_gt], dx_gt[valid_gt])
            cog_rad_gt = (cog_angles_gt + np.pi) % (2.0 * np.pi) - np.pi
            cog_smooth_lookup = (valid_t_gt, cog_rad_gt)

            # Plot for Yaw comparison
            cog_t_plot, cog_y_plot = clean_for_plotting(valid_t_gt, np.degrees(cog_rad_gt))
            ax_yaw.plot(cog_t_plot, cog_y_plot, 'b-', lw=2, alpha=0.8, label='Course Truth (Raw Blue)')

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
    ax_vel.set_ylabel('Velocity (m/s)')
    ax_vel.set_xlabel('Time (s)')
    ax_vel.set_title('Velocity & Acceleration Profile')
    ax_vel.grid(True)
    ax_vel.axhline(0.1, color='m', linestyle='--', alpha=0.5, label='Fusion Threshold (0.1m/s)')
    
    # --- Acceleration Plot (Twin Axis) ---
    ax_accel = ax_vel.twinx()
    # Index 7 is Acceleration
    ax_accel.plot(data[:, 0], data[:, 7], 'tab:red', alpha=0.6, lw=1.5, label='Accel (m/s²)')
    ax_accel.set_ylabel('Accel (m/s²)', color='tab:red')
    ax_accel.tick_params(axis='y', labelcolor='tab:red')
    
    # Moving markers
    vel_marker, = ax_vel.plot([], [], 'ko', ms=6, label='Current Speed')
    accel_marker, = ax_accel.plot([], [], 'ro', ms=6, label='Current Accel')
    
    # Combine legends
    lines_v, labels_v = ax_vel.get_legend_handles_labels()
    lines_a, labels_a = ax_accel.get_legend_handles_labels()
    ax_vel.legend(lines_v + lines_a, labels_v + labels_a, loc='upper left', fontsize='small')

    # Pre-calculate errors for all decimated frames to find global min/max
    errors_ekf = []
    errors_raw = []
    extremes = {} # {frame_idx: "Label"}
    
    if cog_smooth_lookup is not None:
        t_gt, yaw_gt = cog_smooth_lookup
        def angular_diff_local(a, b):
            return np.abs((a - b + np.pi) % (2.0 * np.pi) - np.pi)
            
        for i in range(len(data_dec)):
            ti = data_dec[i, 0]
            yawi = data_dec[i, 4]
            raw_yawi_ned = data_dec[i, 8]
            raw_yawi_enu = (np.pi/2.0 - raw_yawi_ned + np.pi) % (2 * np.pi) - np.pi
            
            idx_gt = np.searchsorted(t_gt, ti)
            if idx_gt > 0 and idx_gt < len(t_gt):
                truth_yawi = yaw_gt[idx_gt]
                errors_ekf.append(np.degrees(angular_diff_local(yawi, truth_yawi)))
                errors_raw.append(np.degrees(angular_diff_local(raw_yawi_enu, truth_yawi)))
            else:
                errors_ekf.append(0.0)
                errors_raw.append(0.0)
                
        if len(errors_ekf) > 0:
            idx_ekf_max = np.argmax(errors_ekf)
            idx_ekf_min = np.argmin(errors_ekf)
            idx_raw_max = np.argmax(errors_raw)
            idx_raw_min = np.argmin(errors_raw)
            
            extremes[idx_ekf_max] = f"EKF MAX ERROR ({errors_ekf[idx_ekf_max]:.1f}°)"
            extremes[idx_ekf_min] = f"EKF MIN ERROR ({errors_ekf[idx_ekf_min]:.2f}°)"
            extremes[idx_raw_max] = f"RAW MAX ERROR ({errors_raw[idx_raw_max]:.1f}°)"
            # Skip Raw Min to avoid clutter as EKF Min is often close
    
    # Text markers for extremes (initially hidden)
    extreme_marker = ax_map.text(0, 0, "", color='white', weight='bold', 
                                bbox=dict(facecolor='black', alpha=0.8, boxstyle='round,pad=0.3'),
                                zorder=10, visible=False)

    # Cumulative Error Tracking
    cumulative_ekf_err = []
    cumulative_raw_err = []

    def update(i):
        # Easting (X), Northing (Y)
        ei = data_dec[i, 1] 
        ni = data_dec[i, 2] 
        yawi = data_dec[i, 4] # Corrected
        # Index 8 is Raw NED
        raw_yawi_ned = data_dec[i, 8]
        raw_yawi = np.pi/2.0 - raw_yawi_ned
        raw_yawi = (raw_yawi + np.pi) % (2 * np.pi) - np.pi
        
        ti = data_dec[i, 0]
        vi = data_dec[i, 3] # Velocity
        ai = data_dec[i, 7] # Accel
        
        # Trail
        trail_line.set_data(data_dec[:i+1, 1], data_dec[:i+1, 2])
        
        # Marker (Map)
        robot_marker.set_data([ei], [ni])
        
        # Markers
        vel_marker.set_data([ti], [vi])
        accel_marker.set_data([ti], [ai])
        
        # Line (Yaw)
        yaw_time_line.set_xdata([ti, ti])
        
        # Heading Vectors (20m)
        len_vec = 20.0
        
        # 1. Corrected (Green)
        U = len_vec * np.cos(yawi)
        V = len_vec * np.sin(yawi)
        arrow_line.set_data([ei, ei+U], [ni, ni+V])
        
        # raw_yawi is already converted to ENU above
        raw_yaw_enu = raw_yawi
        
        U_raw = len_vec * np.cos(raw_yaw_enu)
        V_raw = len_vec * np.sin(raw_yaw_enu)
        arrow_raw_line.set_data([ei, ei+U_raw], [ni, ni+V_raw])
        
        # 3. Course Over Ground (Blue) - "Truth" from movement
        if i > 0:
            # Look back 1 frame (or more if needed)
            prev_ei = data_dec[i-1, 1]
            prev_ni = data_dec[i-1, 2]
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
            
        # 4. Error Calculation
        err_str = ""
        if cog_smooth_lookup is not None:
            t_gt, yaw_gt = cog_smooth_lookup
            # Find closest GT point
            idx_gt = np.searchsorted(t_gt, ti)
            if idx_gt > 0 and idx_gt < len(t_gt):
                truth_yawi = yaw_gt[idx_gt]
                
                def angular_diff(a, b):
                    return np.abs((a - b + np.pi) % (2.0 * np.pi) - np.pi)
                
                ekf_err = np.degrees(angular_diff(yawi, truth_yawi))
                raw_err = np.degrees(angular_diff(raw_yaw_enu, truth_yawi))
                
                cumulative_ekf_err.append(ekf_err)
                cumulative_raw_err.append(raw_err)
                
                mae_ekf = np.mean(cumulative_ekf_err)
                mae_raw = np.mean(cumulative_raw_err)
                
                err_str = f" | EKF Err: {ekf_err:.1f}° (MAE: {mae_ekf:.1f}°) | Raw Err: {raw_err:.1f}° (MAE: {mae_raw:.1f}°)"

        # Highlight Extremes (Persistence for 60 frames / 2 seconds)
        duration = 60
        found_active = False
        for start_idx, label in extremes.items():
            if start_idx <= i < start_idx + duration:
                # Update text and position to where it happened
                ex_ei = data_dec[start_idx, 1]
                ex_ni = data_dec[start_idx, 2]
                extreme_marker.set_text(label)
                extreme_marker.set_position((ex_ei + 10, ex_ni + 10))
                extreme_marker.set_visible(True)
                found_active = True
                break
        
        if not found_active:
            extreme_marker.set_visible(False)

        time_text.set_text(f'Time: {ti:.1f}s | Yaw: {np.degrees(yawi):.1f} deg | Vel: {vi:.2f} m/s' + err_str)
        
        # Print Debug Info
        if i % 10 == 0 or i == len(data_dec) - 1:
            # Print Raw in ENU for comparison
            raw_deg_enu = np.degrees(raw_yaw_enu)
            # Wrap to [-180, 180] for readability
            raw_deg_enu = (raw_deg_enu + 180) % 360 - 180
            
            ekf_deg = np.degrees(yawi)
            cog_str = "NaN"
            if 'cog_angle' in locals():
                cog_str = f"{np.degrees(cog_angle):.1f}"
            
            print(f"Frame {i}: Time={ti:.1f}s | EKF={ekf_deg:.1f} | Raw(ENU)={raw_deg_enu:.1f} | Course={cog_str}{err_str}", flush=True)

        return trail_line, robot_marker, arrow_line, arrow_raw_line, arrow_cog_line, time_text, vel_marker, yaw_time_line, accel_marker, extreme_marker


    ani = animation.FuncAnimation(fig, update, frames=len(data_dec), interval=30, blit=True)
    
    print(f"Rendering {len(data_dec)} frames... this may take a moment...", flush=True)
    print(f"Saving animation to {SAVE_PATH} ...", flush=True)
    ani.save(SAVE_PATH, writer='pillow', fps=30)
    print("Animation saved!", flush=True)

if __name__ == "__main__":
    animate_results()
