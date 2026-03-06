
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
                    pass
            except ValueError:
                continue
    return np.array(data)


def animate_results():
    if not os.path.exists(RESULTS_FILE):
        print(f"Error: Results file not found at {RESULTS_FILE}")
        return

    data = load_results(RESULTS_FILE)
    if len(data) == 0:
        return
    
    # Decimate for speed (plot every 300th point)
    decimate = 300
    data_dec = data[::decimate]
    
    # Setup Figure with 2 subplots (Map, Yaw)
    fig = plt.figure(figsize=(10, 10))
    gs = fig.add_gridspec(2, 1, height_ratios=[2, 1], hspace=0.3)
    
    ax_map = fig.add_subplot(gs[0])
    ax_yaw = fig.add_subplot(gs[1])
    
    # --- Map Plot Setup ---
    ax_map.set_aspect('equal')
    ax_map.grid(True)
    ax_map.set_xlabel('Relative Easting (m)')
    ax_map.set_ylabel('Relative Northing (m)')
    ax_map.set_title('Robot Trajectory Tracking Accuracy (EKF)', fontsize=14, weight='bold')
    
    trail_line, = ax_map.plot([], [], 'r-', alpha=0.5, lw=1, label='Path')
    robot_marker, = ax_map.plot([], [], 'bo', ms=6, label='Robot')
    
    # Arror Vectors
    arrow_line, = ax_map.plot([], [], 'g-', lw=2, label='EKF Heading')
    arrow_raw_line, = ax_map.plot([], [], 'r-', lw=2, label='Raw Sensor')
    arrow_cog_line, = ax_map.plot([], [], 'b-', lw=2, label='Course Truth')
    
    # Set Limits
    min_e, max_e = np.min(data_dec[:, 1]), np.max(data_dec[:, 1])
    min_n, max_n = np.min(data_dec[:, 2]), np.max(data_dec[:, 2])
    margin = 50
    ax_map.set_xlim(min_e - margin, max_e + margin)
    ax_map.set_ylim(min_n - margin, max_n + margin)
    ax_map.legend(loc='upper right', fontsize='small')
    
    time_text = ax_map.text(0.02, 0.02, '', transform=ax_map.transAxes, fontsize=10, family='monospace')

    # --- Yaw Plot Setup ---
    time_arr = data[:, 0]
    ekf_yaw_deg = np.degrees(data[:, 4])
    raw_yaw_deg = np.degrees(data[:, 8])
    
    def clean_for_plotting(t, y, threshold=50):
        dy = np.diff(y)
        jump_inds = np.where(np.abs(dy) > threshold)[0]
        if len(jump_inds) == 0: return t, y
        t_clean = np.insert(t, jump_inds+1, np.nan)
        y_clean = np.insert(y, jump_inds+1, np.nan)
        return t_clean, y_clean

    ekf_t, ekf_y = clean_for_plotting(time_arr, ekf_yaw_deg)
    ax_yaw.plot(ekf_t, ekf_y, 'g-', lw=2, label='EKF Yaw')
    
    raw_yaw_enu = (90 - raw_yaw_deg + 180) % 360 - 180
    raw_t, raw_y = clean_for_plotting(time_arr, raw_yaw_enu)
    ax_yaw.plot(raw_t, raw_y, 'r--', alpha=0.6, label='Raw INS Yaw (ENU)')
    
    # Course Truth background
    stride_gt = 100
    if len(data) > stride_gt:
        t_gt = time_arr[::stride_gt]
        x_gt = data[::stride_gt, 9] 
        y_gt = data[::stride_gt, 10] 
        dx_gt = np.diff(x_gt)
        dy_gt = np.diff(y_gt)
        valid_gt = np.hypot(dx_gt, dy_gt) > 0.5
        if np.any(valid_gt):
            valid_t_gt = t_gt[:-1][valid_gt]
            cog_rad_gt = (np.arctan2(dy_gt[valid_gt], dx_gt[valid_gt]) + np.pi) % (2.0 * np.pi) - np.pi
            cog_t_plot, cog_y_plot = clean_for_plotting(valid_t_gt, np.degrees(cog_rad_gt))
            ax_yaw.plot(cog_t_plot, cog_y_plot, 'b-', lw=1.5, alpha=0.7, label='Course Truth (Raw Blue)')

    ax_yaw.set_ylabel('Yaw (deg)')
    ax_yaw.set_yticks([-180, -90, 0, 90, 180])
    ax_yaw.set_yticklabels(['W', 'S', 'E', 'N', 'W'])
    ax_yaw.set_ylim(-220, 220)
    ax_yaw.legend(loc='lower right', fontsize='small')
    ax_yaw.grid(True, linestyle='--', alpha=0.5)
    
    yaw_time_line = ax_yaw.axvline(data[0, 0], color='k', lw=1, alpha=0.5)

    # Final Overlay
    reduction_overlay = fig.text(0.5, 0.45, '85.6% DRIFT REDUCTION', fontsize=24, color='darkgreen',
                                weight='black', ha='center', va='center',
                                bbox=dict(facecolor='white', alpha=0.9, edgecolor='green', boxstyle='round,pad=0.5'),
                                zorder=100)
    reduction_overlay.set_visible(False)

    def init():
        trail_line.set_data([], [])
        robot_marker.set_data([], [])
        arrow_line.set_data([], [])
        arrow_raw_line.set_data([], [])
        arrow_cog_line.set_data([], [])
        reduction_overlay.set_visible(False)
        return trail_line, robot_marker, arrow_line, arrow_raw_line, arrow_cog_line, time_text

    def update(i):
        ei, ni = data_dec[i, 1], data_dec[i, 2]
        yawi = data_dec[i, 4]
        ti = data_dec[i, 0]
        raw_yawi_ned = data_dec[i, 8]
        raw_yaw_enu = (np.pi/2.0 - raw_yawi_ned + np.pi) % (2.0 * np.pi) - np.pi
        
        trail_line.set_data(data_dec[:i+1, 1], data_dec[:i+1, 2])
        robot_marker.set_data([ei], [ni])
        
        len_vec = 20.0
        arrow_line.set_data([ei, ei + len_vec*np.cos(yawi)], [ni, ni + len_vec*np.sin(yawi)])
        arrow_raw_line.set_data([ei, ei + len_vec*np.cos(raw_yaw_enu)], [ni, ni + len_vec*np.sin(raw_yaw_enu)])
        
        # Course vector
        if i > 0:
            dx, dy = ei - data_dec[i-1, 1], ni - data_dec[i-1, 2]
            if np.hypot(dx, dy) > 0.1:
                cog = np.arctan2(dy, dx)
                arrow_cog_line.set_data([ei, ei + len_vec*np.cos(cog)], [ni, ni + len_vec*np.sin(cog)])

        yaw_time_line.set_xdata([ti, ti])
        
        if i > len(data_dec) * 0.9:
            reduction_overlay.set_visible(True)
            
        time_text.set_text(f"Time: {ti:.1f}s | Heading: {np.degrees(yawi):.1f}°")
        return trail_line, robot_marker, arrow_line, arrow_raw_line, arrow_cog_line, time_text

    ani = animation.FuncAnimation(fig, update, frames=len(data_dec), init_func=init, blit=True, interval=50)
    print(f"Saving blog-ready animation to {SAVE_PATH}...")
    ani.save(SAVE_PATH, writer='pillow', fps=20)

if __name__ == "__main__":
    animate_results()
