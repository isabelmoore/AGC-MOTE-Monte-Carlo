
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import utm
from datetime import datetime
import sys
import os

# Add src to python path to import the official filter
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/mote_ros/kalman_filter')))
from kalman_filter import KalmanFilter
import pickle

def analyze_bag(bag_path):
    cache_file = os.path.abspath(os.path.join(os.path.dirname(__file__), '../results/bag_cache.pkl'))
    
    # Check if cache exists
    if os.path.exists(cache_file):
        print(f"Loading data from cache: {cache_file}")
        with open(cache_file, 'rb') as f:
            events = pickle.load(f)
        print(f"Loaded {len(events)} events from cache.")
    else:
        print(f"Opening bag: {bag_path}")
        if not os.path.exists(bag_path):
            print(f"Error: Bag file not found at {bag_path}")
            return
            
        bag = rosbag.Bag(bag_path)
        
        # Events: ('INS', time, msg) or ('TWIST', time, msg)
        events = []
        
        # Read messages
        print("Reading topics (this may take a while)...")
        start_time = None
        for topic, msg, t in bag.read_messages(topics=['/vectornav/INS', '/vehicle/twist']):
            if start_time is None:
                start_time = t.to_sec()
            
            rel_time = t.to_sec() - start_time
            if topic == '/vectornav/INS':
                # Store primitives, not the object, for pickle safety
                events.append({
                    'type': 'INS', 
                    't': rel_time, 
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'yaw': msg.yaw
                })
            elif topic == '/vehicle/twist':
                events.append({
                    'type': 'TWIST', 
                    't': rel_time, 
                    'linear_x': msg.twist.linear.x,
                    'angular_z': msg.twist.angular.z
                })

        bag.close()
        
        # Sort events by time
        events.sort(key=lambda x: x['t'])
        
        # Save to cache
        os.makedirs(os.path.dirname(cache_file), exist_ok=True)
        print(f"Saving extracted data to cache: {cache_file}")
        with open(cache_file, 'wb') as f:
            pickle.dump(events, f)
        
        print(f"Extracted and saved {len(events)} events.")
    
    if not events:
        print("No data found!")
        return

    # Initialize Filter
    kf = KalmanFilter()
    
    # Find first INS message for initialization
    first_ins = next((e for e in events if e['type'] == 'INS'), None)
    if not first_ins:
        print("No INS data for initialization!")
        return
        
    init_msg = first_ins
    init_lat, init_lon = init_msg['latitude'], init_msg['longitude']
    init_utm_e, init_utm_n, _, _ = utm.from_latlon(init_lat, init_lon)
    
    # Initialize State: [E, N, v, theta, rate_bias, head_bias, a]
    # Yaw conversion: NED (0=N, CW) -> ENU (0=E, CCW)
    init_yaw_rad = np.radians(init_msg['yaw'])
    init_yaw_std = np.pi/2.0 - init_yaw_rad
    init_yaw_std = (init_yaw_std + np.pi) % (2.0 * np.pi) - np.pi
    
    # [E, N, v, theta, rate_bias, head_bias, a]
    kf.x = np.array([[init_utm_e], [init_utm_n], [0.0], [init_yaw_std], [0.0], [0.0], [0.0]])
    kf.P[5, 5] = 0.001 # SYNC: Tighten bias uncertainty at start
    print(f"Initialized at E:{init_utm_e:.2f}, N:{init_utm_n:.2f}, Yaw:{init_yaw_std:.2f}")

    # Initialize Matrices
    kf.R_pos = np.eye(2) * 15.0 # ULTRA smoothing for jittery GPS
    kf.R_vel = np.eye(1) * 1.0
    kf.R_yaw = np.eye(1) * 0.1  # SYNC: Match live node (was 0.5)
    # Noise Tuning - Phase 7: Ultra-Stability
    kf.Q[3, 3] = 0.0001     # Tightest heading (Heavy inertia)
    kf.Q[4, 4] = 0.000001   # Ultra-stable rate bias
    kf.Q[5, 5] = 0.0000001  # Ultra-stable heading bias
    kf.Q[6, 6] = 0.01       # Reduced accel noise for smoothness
    
    # History Storage
    hist_est = []
    hist_gps = []
    hist_ins_yaw = []
    innovations_yaw = [] # PHASE 7: Jitter Tracking
    
    last_t = events[0]['t']
    current_yaw_rate = 0.0
    current_linear_v = 0.0
    
    # --- Tuning Parameters ---   
    velocity_fusion_threshold = 0.1 
    turn_fusion_threshold = 0.05    
    min_geometry_distance = 0.15    
    course_fusion_trust = 0.2       # ULTRA de-weight of jumpy Course
    # -------------------------
    
    print("\n--- Phase 7 Tuning Parameters (Ultra-Smoothing) ---")
    print(f"Velocity Fusion Threshold: {velocity_fusion_threshold} m/s")
    print(f"Course Fusion Trust (R):   {course_fusion_trust}")
    print(f"Heading Process Noise (Q): {kf.Q[3,3]}")
    print("---------------------------------------------------\n")
    
    for i, event in enumerate(events):
        t = event['t']
        dt = t - last_t
        last_t = t
        
        # 1. Sync Prediction: Call every frame like the live node
        if dt > 0:
            kf.dt = dt
            kf.predict_ekf(omega_measured=current_yaw_rate)

        if event['type'] == 'TWIST':
            current_linear_v = event['linear_x']
            current_yaw_rate = event['angular_z']
            
            z_vel = np.array([[current_linear_v]])
            kf.update(z_vel, kf.H_vel, kf.R_vel)
            
            # --- Zero-Velocity Constraint ---
            if abs(current_linear_v) < 0.01:
                kf.x[2, 0] = 0.0 
                kf.x[6, 0] = 0.0 
            
        elif event['type'] == 'INS':
            # 1. Position Update
            lat, lon = event['latitude'], event['longitude']
            utm_e, utm_n, _, _ = utm.from_latlon(lat, lon)
            z_pos = np.array([[utm_e], [utm_n]]) 
            kf.update(z_pos, kf.H_pos, kf.R_pos)
            
            # 3. Process Yaw (Sensor)
            yaw_rad = np.radians(event['yaw'])
            yaw_std = np.pi/2.0 - yaw_rad
            yaw_std = (yaw_std + np.pi) % (2.0 * np.pi) - np.pi
            
            # 4. Course Fusion
            stride = 50
            is_fusing_course = False
            
            if len(hist_gps) > stride:
                curr_e, curr_n = hist_gps[-1][1], hist_gps[-1][2]
                past_e, past_n = hist_gps[-stride][1], hist_gps[-stride][2]
                de, dn = curr_e - past_e, curr_n - past_n
                dist = np.hypot(de, dn)
                est_v = kf.x[2, 0]
                
                if dist > min_geometry_distance and (abs(est_v) > velocity_fusion_threshold or abs(current_yaw_rate) > turn_fusion_threshold):
                    gps_course = np.arctan2(dn, de) 
                    if est_v < -0.1:
                        gps_course = (gps_course + np.pi + np.pi) % (2.0 * np.pi) - np.pi
                    
                    is_fusing_course = True
                    z_course = np.array([[gps_course]])
                    innov = kf.update(z_course, kf.H_yaw, np.eye(1)*course_fusion_trust, angle_indices=[0])
                    innovations_yaw.append(abs(innov[0, 0]))

            # 5. SENSOR UPDATE
            z_yaw = np.array([[yaw_std]])
            if is_fusing_course:
                r_sensor = 100.0 # Pushed back even more
            else:
                is_stopped = abs(current_linear_v) < 0.02
                r_sensor = 200.0 if is_stopped else 20.0 # Heavier filter
                
            innov = kf.update(z_yaw, kf.H_sensor, np.eye(1)*r_sensor, angle_indices=[0])
            innovations_yaw.append(abs(innov[0, 0]))
            
            # Store History
            hist_gps.append([t, utm_e, utm_n])
            yaw_raw_ned = yaw_rad
            state = kf.x.flatten()
            hist_est.append([t, state[0], state[1], state[2], state[3], state[4], state[5], state[6], yaw_raw_ned])
            hist_ins_yaw.append([t, yaw_std])

    # ANALYTICS: Jitter & Error Metrics
    err_metrics = {}
    if len(innovations_yaw) > 0:
        rms_jitter_rad = np.sqrt(np.mean(np.square(innovations_yaw)))
        print(f"\n--- JITTER ANALYTICS ---")
        print(f"Heading Innovation RMS: {rms_jitter_rad:.6f} rad ({np.degrees(rms_jitter_rad):.4f} deg)")
        print(f"Stability Score: {(1.0 - min(rms_jitter_rad, 1.0))*100:.1f}/100")
        print("------------------------\n")

    # Convert to arrays for processing
    hist_est = np.array(hist_est)
    hist_gps = np.array(hist_gps)
    hist_ins_yaw = np.array(hist_ins_yaw)
    time_arr = hist_est[:, 0]

    # --- Precise Error Metrics (vs Ground Truth Course) ---
    stride = 50 # Increased for stable GPS-derived course
    if len(hist_gps) > stride:
        t_sub_gps = hist_gps[::stride, 0] # Save for error timestamps
        x_sub = hist_gps[::stride, 1] # Use Raw GPS East
        y_sub = hist_gps[::stride, 2] # Use Raw GPS North
        
        # We need EKF and Raw Yaw at the SAME strides
        # Since hist_est and hist_gps are synced (appended together), we use the same indices
        ekf_yaw_sub = hist_est[::stride, 4]
        raw_yaw_sub = hist_est[::stride, 8]
        
        dx = np.diff(x_sub)
        dy = np.diff(y_sub)
        dist = np.hypot(dx, dy)
        valid_mask = dist > 0.5
        
        if np.any(valid_mask):
            cog_angles = np.arctan2(dy[valid_mask], dx[valid_mask])
            cog_rad = (cog_angles + np.pi) % (2.0 * np.pi) - np.pi
            
            # Subsample EKF and Raw to match valid COG points
            ekf_yaw_valid = ekf_yaw_sub[:-1][valid_mask]
            raw_yaw_ned_valid = raw_yaw_sub[:-1][valid_mask]
            raw_yaw_enu_valid = (np.pi/2.0 - raw_yaw_ned_valid + np.pi) % (2.0 * np.pi) - np.pi
            
            # Calculate Angular Errors
            def angular_diff(a, b):
                diff = (a - b + np.pi) % (2.0 * np.pi) - np.pi
                return np.abs(diff)
            
            ekf_errors = angular_diff(ekf_yaw_valid, cog_rad)
            raw_errors = angular_diff(raw_yaw_enu_valid, cog_rad)
            
            err_metrics['ekf_mae_deg'] = np.degrees(np.mean(ekf_errors))
            err_metrics['ekf_std_deg'] = np.degrees(np.std(ekf_errors))
            err_metrics['ekf_max_deg'] = np.degrees(np.max(ekf_errors))
            err_metrics['ekf_min_deg'] = np.degrees(np.min(ekf_errors))
            
            # Robust Metrics (95% CI and Trimmed Mean)
            err_metrics['ekf_95ci'] = np.degrees(np.percentile(ekf_errors, [2.5, 97.5]))
            ekf_mask_95 = (ekf_errors >= np.percentile(ekf_errors, 2.5)) & (ekf_errors <= np.percentile(ekf_errors, 97.5))
            err_metrics['ekf_mae_95_deg'] = np.degrees(np.mean(ekf_errors[ekf_mask_95]))
            
            # Identify timestamps for extremes
            t_valid = t_sub_gps[:-1][valid_mask]
            idx_ekf_max = np.argmax(ekf_errors)
            idx_ekf_min = np.argmin(ekf_errors)
            err_metrics['ekf_max_t'] = t_valid[idx_ekf_max]
            err_metrics['ekf_max_y'] = np.degrees(ekf_yaw_valid[idx_ekf_max])
            err_metrics['ekf_min_t'] = t_valid[idx_ekf_min]
            err_metrics['ekf_min_y'] = np.degrees(ekf_yaw_valid[idx_ekf_min])
            
            err_metrics['raw_mae_deg'] = np.degrees(np.mean(raw_errors))
            err_metrics['raw_std_deg'] = np.degrees(np.std(raw_errors))
            err_metrics['raw_max_deg'] = np.degrees(np.max(raw_errors))
            err_metrics['raw_min_deg'] = np.degrees(np.min(raw_errors))
            
            # Robust Metrics (Raw)
            err_metrics['raw_95ci'] = np.degrees(np.percentile(raw_errors, [2.5, 97.5]))
            raw_mask_95 = (raw_errors >= np.percentile(raw_errors, 2.5)) & (raw_errors <= np.percentile(raw_errors, 97.5))
            err_metrics['raw_mae_95_deg'] = np.degrees(np.mean(raw_errors[raw_mask_95]))
            
            idx_raw_max = np.argmax(raw_errors)
            err_metrics['raw_max_t'] = t_valid[idx_raw_max]
            err_metrics['raw_max_y'] = np.degrees(raw_yaw_enu_valid[idx_raw_max])
            
            print(f"--- FUSION ERROR METRICS (vs Raw Blue) ---")
            print(f"Green (EKF) Mean: {err_metrics['ekf_mae_deg']:.3f}° | Std: {err_metrics['ekf_std_deg']:.3f}° | Range: [{err_metrics['ekf_min_deg']:.3f}, {err_metrics['ekf_max_deg']:.3f}]°")
            print(f"            95% CI: [{err_metrics['ekf_95ci'][0]:.3f}, {err_metrics['ekf_95ci'][1]:.3f}]° | Trimmed Mean (95%): {err_metrics['ekf_mae_95_deg']:.3f}°")
            print(f"Red (Raw)   Mean: {err_metrics['raw_mae_deg']:.3f}° | Std: {err_metrics['raw_std_deg']:.3f}° | Range: [{err_metrics['raw_min_deg']:.3f}, {err_metrics['raw_max_deg']:.3f}]°")
            print(f"            95% CI: [{err_metrics['raw_95ci'][0]:.3f}, {err_metrics['raw_95ci'][1]:.3f}]° | Trimmed Mean (95%): {err_metrics['raw_mae_95_deg']:.3f}°")
            improvement = 100.0 * (err_metrics['raw_mae_deg'] - err_metrics['ekf_mae_deg']) / err_metrics['raw_mae_deg']
            improvement_95 = 100.0 * (err_metrics['raw_mae_95_deg'] - err_metrics['ekf_mae_95_deg']) / err_metrics['raw_mae_95_deg']
            print(f"Drift Reduction: {improvement:.1f}% (95% Trimmed: {improvement_95:.1f}%)")
            print("-----------------------------------------------\n")

    
    # ...
    
    print(f"Processing complete. Steps: {len(hist_est)}")
    
    # Save Results
    results_dir = os.path.join(os.path.dirname(__file__), '../results')
    os.makedirs(results_dir, exist_ok=True)
    
    # 1. Text Output
    res_file = os.path.join(results_dir, 'kalman_filter_results.txt')
    with open(res_file, 'w') as f:
        # Header (REVERTED TO RADIANS): T, E, N, V, Yaw_rad, RateBias_rads, HeadBias_rad, Accel, RawYaw_rad, GPS_E, GPS_N
        f.write('Timestamp, E, N, V, Yaw_rad, RateBias_rads, HeadBias_rad, Accel, RawYaw_rad, GPS_E, GPS_N\n')
        for i in range(len(hist_est)):
            r = hist_est[i]
            g = hist_gps[i]
            # Use original Radian values for code compatibility
            f.write(f"{r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f}, {r[3]:.4f}, {r[4]:.6f}, {r[5]:.8f}, {r[6]:.8f}, {r[7]:.4f}, {r[8]:.6f}, {g[1]:.4f}, {g[2]:.4f}\n")
    print(f"Results saved to {res_file}")

    # 2. Plot
    # Normalize
    if len(hist_gps) > 0:
        # Index 1 is East (x), Index 2 is North (y)
        e0, n0 = hist_gps[0, 1], hist_gps[0, 2]
        hist_gps[:, 1] -= e0
        hist_gps[:, 2] -= n0
        hist_est[:, 1] -= e0
        hist_est[:, 2] -= n0
        
    plt.figure(figsize=(12, 12))
    # hist_gps: [t, E, N]
    plt.plot(hist_gps[:, 1], hist_gps[:, 2], 'k.', alpha=0.3, label='GPS (Zeroed)')
    plt.plot(hist_est[:, 1], hist_est[:, 2], 'b-', lw=1, alpha=0.5, label='EKF Trajectory')
    
    # Add Heading Arrows (Subsampled)
    # Plot every Nth point to avoid clutter
    step = 500 
    if len(hist_est) > step:
        sub_est = hist_est[::step]
        
        # Extract components
        x = sub_est[:, 1] # East
        y = sub_est[:, 2] # North
        yaw_corr = sub_est[:, 4]
        yaw_raw = sub_est[:, 8]
        
        # Vector Length (visual)
        vec_len = 20.0
        
        # Corrected (Green)
        u_corr = vec_len * np.cos(yaw_corr)
        v_corr = vec_len * np.sin(yaw_corr)
        plt.quiver(x, y, u_corr, v_corr, color='g', angles='xy', scale_units='xy', scale=1, width=0.003, label='Corrected Yaw')
        
        # Raw (Red) - Convert NED to ENU
        yaw_raw_enu = np.pi/2.0 - yaw_raw
        u_raw = vec_len * np.cos(yaw_raw_enu)
        v_raw = vec_len * np.sin(yaw_raw_enu)
        plt.quiver(x, y, u_raw, v_raw, color='r', angles='xy', scale_units='xy', scale=1, width=0.003, label='Raw INS Yaw')
        
        # Course Over Ground (Blue) - Data Driven Truth
        # Vector from point i to i+1
        cog_u = np.diff(x)
        cog_v = np.diff(y)
        # Normalize to vec_len
        cog_mag = np.hypot(cog_u, cog_v)
        # Filter static points
        valid = cog_mag > 0.1
        
        if np.any(valid):
            # Normalize
            cog_u[valid] = (cog_u[valid] / cog_mag[valid]) * vec_len
            cog_v[valid] = (cog_v[valid] / cog_mag[valid]) * vec_len
            # Plot at starting points
            plt.quiver(x[:-1][valid], y[:-1][valid], cog_u[valid], cog_v[valid], color='b', angles='xy', scale_units='xy', scale=1, width=0.003, label='Course Truth (Blue)')

    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Offline EKF Trajectory with Heading Vectors')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    
    plot_file = os.path.join(results_dir, 'trajectory_offline.png')
    plt.savefig(plot_file)
    print(f"Plot saved to {plot_file}")
    
    # 3. Plot Yaw Comparison and Velocity
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    
    # Subplot 1: Yaw
    time_arr = hist_est[:, 0]
    ekf_yaw_deg = np.degrees(hist_est[:, 4])
    raw_yaw_deg = np.degrees(hist_est[:, 8])
    
    # Helper to remove vertical lines when wrapping from 180 to -180
    def remove_wrapping_artifacts(t, y, threshold=50): # Tightened for cleaner plots
        t_clean = t.copy()
        y_clean = y.copy()
        
        # Find indices where jump > threshold
        d = np.diff(y_clean)
        idx = np.where(np.abs(d) > threshold)[0]
        
        if len(idx) > 0:
            # Insert NaNs to break the line
            out_t = np.insert(t_clean, idx+1, np.nan)
            out_y = np.insert(y_clean, idx+1, np.nan)
            return out_t, out_y
        return t_clean, y_clean

    # 1. EKF Yaw
    t_ekf, y_ekf = remove_wrapping_artifacts(time_arr, ekf_yaw_deg)
    label_green = 'EKF Yaw (Corrected)'
    if 'ekf_mae_deg' in err_metrics:
        label_green += f' (MAE: {err_metrics["ekf_mae_deg"]:.2f}°)'
    ax1.plot(t_ekf, y_ekf, 'g-', lw=2, label=label_green)

    # 2. Raw INS Yaw (Convert NED -> ENU first)
    raw_yaw_enu_deg = (90 - raw_yaw_deg + 180) % 360 - 180
    t_raw, y_raw = remove_wrapping_artifacts(time_arr, raw_yaw_enu_deg)
    label_red = 'Raw INS Yaw (ENU)'
    if 'raw_mae_deg' in err_metrics:
        label_red += f' (MAE: {err_metrics["raw_mae_deg"]:.2f}°)'
    ax1.plot(t_raw, y_raw, 'r--', alpha=0.6, label=label_red)
    
    # Course Over Ground (Blue) - Use Raw GPS
    stride_gt = 50 
    if len(hist_gps) > stride_gt:
        t_sub_gt = hist_gps[::stride_gt, 0]
        x_sub_gt = hist_gps[::stride_gt, 1] 
        y_sub_gt = hist_gps[::stride_gt, 2] 
        
        dx = np.diff(x_sub_gt)
        dy = np.diff(y_sub_gt)
        dist = np.hypot(dx, dy)
        valid_mask = dist > 0.5
        
        if np.any(valid_mask):
            # Calculate Raw Course angles for plotting
            cog_angles = np.arctan2(dy[valid_mask], dx[valid_mask])
            cog_deg = np.degrees((cog_angles + np.pi) % (2.0 * np.pi) - np.pi)
            
            # Decimate time to match valid COG points
            t_cog_raw = t_sub_gt[:-1][valid_mask]
            
            # Ensure fresh copy for cleaning to avoid scope leakage
            t_cog_clean, y_cog_clean = remove_wrapping_artifacts(t_cog_raw, cog_deg)
            
            label_blue = 'Course Truth (Raw Blue)'
            if 'ekf_mae_deg' in err_metrics:
                label_blue = f'Course Truth (Raw Blue)'
            ax1.plot(t_cog_clean, y_cog_clean, 'b-', lw=1.5, alpha=0.7, label=label_blue)

    # --- Annotate Extremes (Arrows) ---
    if 'ekf_max_t' in err_metrics:
        # EKF Max
        ax1.annotate(f'EKF MAX\n{err_metrics["ekf_max_deg"]:.1f}°', 
                    xy=(err_metrics['ekf_max_t'], err_metrics['ekf_max_y']),
                    xytext=(err_metrics['ekf_max_t'], err_metrics['ekf_max_y'] + 40),
                    arrowprops=dict(facecolor='green', shrink=0.05, width=1, headwidth=5),
                    color='green', weight='bold', ha='center', zorder=10)
        
        # EKF Min
        ax1.annotate(f'EKF MIN\n{err_metrics["ekf_min_deg"]:.2f}°', 
                    xy=(err_metrics['ekf_min_t'], err_metrics['ekf_min_y']),
                    xytext=(err_metrics['ekf_min_t'], err_metrics['ekf_min_y'] - 60),
                    arrowprops=dict(facecolor='darkgreen', shrink=0.05, width=1, headwidth=5),
                    color='darkgreen', weight='bold', ha='center', zorder=10)
        
        # Raw Max
        ax1.annotate(f'RAW MAX\n{err_metrics["raw_max_deg"]:.1f}°', 
                    xy=(err_metrics['raw_max_t'], err_metrics['raw_max_y']),
                    xytext=(err_metrics['raw_max_t'], err_metrics['raw_max_y'] - 40),
                    arrowprops=dict(facecolor='red', shrink=0.05, width=1, headwidth=5),
                    color='red', weight='bold', ha='center', zorder=10)
    
    ax1.set_ylabel('Yaw (deg)')
    ax1.set_title('Yaw Comparison: Corrected vs Raw vs Course')
    
    # Set Cardinal Labels
    ax1.set_yticks([-180, -90, 0, 90, 180])
    ax1.set_yticklabels(['West (-180)', 'South (-90)', 'East (0)', 'North (90)', 'West (180)'])
    ax1.set_ylim(-190, 190)
    
    ax1.legend()
    ax1.grid(True)
    
    # Subplot 2: Velocity
    velo = hist_est[:, 3] # Index 3 is Velocity (v)
    ax2.plot(time_arr, velo, 'k-', lw=1.5, label='Estimated Velocity')
    ax2.axhline(1.0, color='m', linestyle='--', alpha=0.7, label='Fusion Threshold (1.0 m/s)')
    ax2.fill_between(time_arr, 0, 1.0, color='gray', alpha=0.1, label='Static Mode (No Course Fusion)')
    
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Velocity Profile')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    
    plot_yaw_file = os.path.join(results_dir, 'yaw_comparison.png')
    plt.savefig(plot_yaw_file)
    print(f"Yaw Comparison saved to {plot_yaw_file}")

if __name__ == "__main__":
    # Locate bag file relative to script
    bag_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../jeep_loca_pos_2024-05-23-19-40-51.bag'))
    if not os.path.exists(bag_path):
        print(f"Bag file not found at {bag_path}")
    else:
        analyze_bag(bag_path)
