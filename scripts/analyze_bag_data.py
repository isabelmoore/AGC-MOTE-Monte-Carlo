
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
    print(f"Initialized at E:{init_utm_e:.2f}, N:{init_utm_n:.2f}, Yaw:{init_yaw_std:.2f}")

    # Initialize Matrices
    kf.R_pos = np.eye(2) * 15.0 # ULTRA smoothing for jittery GPS
    kf.R_vel = np.eye(1) * 1.0
    kf.R_yaw = np.eye(1) * 0.5  # De-weight sensor slightly more
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
        
        # 1. Predict
        if dt > 0:
            kf.dt = dt
            pass 

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
            # 1. Predict
            kf.predict_ekf(omega_measured=current_yaw_rate) 
            
            # 2. Update Position
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

    # ANALYTICS: Jitter Metrics
    if len(innovations_yaw) > 0:
        rms_jitter = np.sqrt(np.mean(np.square(innovations_yaw)))
        print(f"\n--- JITTER ANALYTICS ---")
        print(f"Heading Innovation RMS: {rms_jitter:.6f} rad ({np.degrees(rms_jitter):.4f} deg)")
        print(f"Stability Score: {(1.0 - min(rms_jitter, 1.0))*100:.1f}/100")
        print("------------------------\n")

    # Convert and Plot
    hist_est = np.array(hist_est)
    hist_gps = np.array(hist_gps)
    hist_ins_yaw = np.array(hist_ins_yaw)
    
    # ...
    
    print(f"Processing complete. Steps: {len(hist_est)}")
    
    # Save Results
    results_dir = os.path.join(os.path.dirname(__file__), '../results')
    os.makedirs(results_dir, exist_ok=True)
    
    # 1. Text Output
    res_file = os.path.join(results_dir, 'kalman_filter_results.txt')
    with open(res_file, 'w') as f:
        # Update Header: T, E, N, V, Yaw, RateBias, HeadBias, Accel, RawYaw
        f.write('Timestamp, E, N, V, Yaw, RateBias, HeadBias, Accel, RawYaw\n')
        for row in hist_est:
            # Row has 9 elements
            f.write(f"{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}, {row[3]:.4f}, {row[4]:.4f}, {row[5]:.4f}, {row[6]:.4f}, {row[7]:.4f}, {row[8]:.4f}\n")
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
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # Subplot 1: Yaw
    time_arr = hist_est[:, 0]
    ekf_yaw_deg = np.degrees(hist_est[:, 4])
    raw_yaw_deg = np.degrees(hist_est[:, 8])
    
    # Helper to remove vertical lines when wrapping from 180 to -180
    def remove_wrapping_artifacts(t, y, threshold=180):
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
    ax1.plot(t_ekf, y_ekf, 'g-', lw=2, label='EKF Yaw (Corrected)')

    # 2. Raw INS Yaw (Convert NED -> ENU first)
    # Original: NED (0=North, CW). ENU (0=East, CCW)
    # Formula: ENU = (90 - NED + 180) % 360 - 180
    raw_yaw_enu = (90 - raw_yaw_deg + 180) % 360 - 180
    
    t_raw, y_raw = remove_wrapping_artifacts(time_arr, raw_yaw_enu)
    ax1.plot(t_raw, y_raw, 'r--', alpha=0.6, label='Raw INS Yaw (ENU)')
    
    # Course Over Ground (Blue) - Line Plot
    # Calculate for all points using a stride to filter noise/low-speed
    stride = 50 # Every 50 samples (~0.5s if 100Hz)
    if len(hist_est) > stride:
        t_sub = time_arr[::stride]
        x_sub = hist_est[::stride, 1] # East
        y_sub = hist_est[::stride, 2] # North
        
        dx = np.diff(x_sub)
        dy = np.diff(y_sub)
        dist = np.hypot(dx, dy)
        
        # Only calculate angles where moving > 0.5m over the stride
        valid_mask = dist > 0.5
        
        if np.any(valid_mask):
            cog_angles = np.arctan2(dy[valid_mask], dx[valid_mask])
            cog_deg = np.degrees(cog_angles)
            
            # Clean up Course lines too
            t_cog = t_sub[:-1][valid_mask]
            
            t_cog_clean, y_cog_clean = remove_wrapping_artifacts(t_cog, cog_deg)
            
            # Plot corresponding times (offset by 1 due to diff)
            ax1.plot(t_cog_clean, y_cog_clean, 'b-', lw=1.5, alpha=0.7, label='Course Truth (Blue)')
    
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
