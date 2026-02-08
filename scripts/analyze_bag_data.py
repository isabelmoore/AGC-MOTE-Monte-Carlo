
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
    
    # Initialize State: [E, N, v, theta, bias]
    # Yaw conversion: NED (0=N, CW) -> ENU (0=E, CCW)
    init_yaw_rad = np.radians(init_msg['yaw'])
    init_yaw_std = np.pi/2.0 - init_yaw_rad
    init_yaw_std = (init_yaw_std + np.pi) % (2.0 * np.pi) - np.pi
    
    kf.x = np.array([[init_utm_e], [init_utm_n], [0.0], [init_yaw_std], [0.0]])
    print(f"Initialized at E:{init_utm_e:.2f}, N:{init_utm_n:.2f}, Yaw:{init_yaw_std:.2f}")

    # Initialize Matrices (matching node usage)
    kf.R_pos = np.eye(2) * 1.0
    kf.R_vel = np.eye(1) * 0.1
    kf.R_yaw = np.eye(1) * 0.1
    # Very stable bias to bridge gaps
    kf.Q[4, 4] = 0.00001 
    
    # Define H_yawrate if not present
    if not hasattr(kf, 'H_yawrate'):
        kf.H_yawrate = np.zeros((1, 5))
        kf.H_yawrate[0, 4] = 1 # Update bias? Or Update yaw rate? 
        # Wait, yaw rate = (yaw - prev_yaw) / dt... 
        # If we measure yaw rate directly (imu), we can update bias.
        # Predicted Yaw Rate = (Not in state directly?)
        # Let's assume we skip yawrate update or map it to bias?
        # Node usage: self.kf.update(z_yawrate, self.kf.H_yawrate, self.kf.R_yawrate)
        # Check node for H_yawrate definition? 
        # I'll enable it but might be zeros.
        pass

    # History Storage
    hist_est = []
    hist_gps = []
    hist_ins_yaw = []
    
    last_t = events[0]['t']
    
    # Process Loop
    current_yaw_rate = 0.0
    # --- Tuning Parameters ---   
    velocity_fusion_threshold = 0.1
    turn_fusion_threshold = 0.05 
    min_geometry_distance = 0.1
    course_fusion_trust = 0.002 # MUCH stronger trust to bridge the gap
    # -------------------------
    
    print("\n--- Current Tuning Parameters ---")
    print(f"Velocity Fusion Threshold: {velocity_fusion_threshold} m/s")
    print(f"Turn Fusion Threshold:     {turn_fusion_threshold} rad/s (~5.7 deg/s)")
    print(f"Min Geometry distance:     {min_geometry_distance} m")
    print(f"Course Fusion Trust (R):   {course_fusion_trust}")
    print("---------------------------------\n")
    
    for i, event in enumerate(events):
        t = event['t']
        dt = t - last_t
        last_t = t
        
        # 1. Predict (if time advanced)
        if dt > 0:
            kf.dt = dt
            # Propagate
            # Use stored yaw rate for prediction if needed, but the filter class might handle it
            pass 

        if event['type'] == 'TWIST':
            linear_x = event['linear_x']
            angular_z = event['angular_z']
            current_yaw_rate = angular_z
            
            z_vel = np.array([[linear_x]])
            kf.update(z_vel, kf.H_vel, kf.R_vel)
            
        elif event['type'] == 'INS':
            # 1. Predict (Advance Time) - USE CURRENT YAW RATE
            kf.predict_ekf(omega_measured=current_yaw_rate) 
            
            # 2. Update Position (GPS)
            lat, lon = event['latitude'], event['longitude']
            utm_e, utm_n, _, _ = utm.from_latlon(lat, lon)
            z_pos = np.array([[utm_e], [utm_n]]) # ENU: East, North
            kf.update(z_pos, kf.H_pos, kf.R_pos)
            
            # 3. Process Yaw (Sensor)
            yaw_rad = np.radians(event['yaw'])
            yaw_std = np.pi/2.0 - yaw_rad
            yaw_std = (yaw_std + np.pi) % (2.0 * np.pi) - np.pi
            
            # 4. Course Fusion & Coasting Logic
            stride = 50
            is_fusing_course = False
            gps_course = None
            
            if len(hist_gps) > stride:
                curr_n, curr_e = hist_gps[-1][1], hist_gps[-1][2]
                past_n, past_e = hist_gps[-stride][1], hist_gps[-stride][2]
                dn = curr_n - past_n
                de = curr_e - past_e
                dist = np.hypot(dn, de)
                est_v = kf.x[2, 0]
                
                moved_enough = dist > min_geometry_distance
                is_fast = est_v > velocity_fusion_threshold
                is_turning = abs(current_yaw_rate) > turn_fusion_threshold
                
                if moved_enough and (is_fast or is_turning):
                    gps_course = np.arctan2(dn, de)
                    is_fusing_course = True
                    
                    # FUSE TRUTH (Strong Anchor)
                    z_course = np.array([[gps_course]])
                    kf.update(z_course, kf.H_yaw, np.eye(1)*course_fusion_trust, angle_indices=[0])

            # 5. SENSOR UPDATE (Red Line Handling)
            z_yaw = np.array([[yaw_std]])
            if is_fusing_course:
                # While fusing truth, treat the red line as weak background
                r_yaw_active = 1.0
            else:
                # GAPS/COASTING: Truth is gone. 
                # Trust the Red Line ALMOST ZERO (R=100) so we don't snap back.
                # The filter will "coast" on its stable bias + gyro.
                r_yaw_active = 100.0
                
            kf.update(z_yaw, kf.H_yaw, np.eye(1)*r_yaw_active, angle_indices=[0])
            
            # Store for history
            hist_gps.append([t, utm_e, utm_n]) # Store E, N
            
            # 4. Course Fusion
            stride = 50
            gps_course = None
            if len(hist_gps) > stride:
                curr_n, curr_e = hist_gps[-1][1], hist_gps[-1][2]
                past_n, past_e = hist_gps[-stride][1], hist_gps[-stride][2]
                dn = curr_n - past_n
                de = curr_e - past_e
                dist = np.hypot(dn, de)
                
                est_v = kf.x[2, 0]
                
                # Dynamic Logic Restored & Tuned:
                # 1. Geometry Check: Moved > 0.15m (15cm) in ~0.5s.
                # 2. Activity Check: Fuse if Fast (>0.3m/s) OR Turning (YawRate > 0.1).
                
                moved_enough = dist > 0.15
                is_fast = est_v > 0.3
                is_turning = abs(current_yaw_rate) > 0.1 
                
                if moved_enough and (is_fast or is_turning):
                    gps_course = np.arctan2(dn, de) # ENU: atan2(North, East)
                    
                    # FUSE IT!
                    z_course = np.array([[gps_course]])
                    # Trust this update SUPER heavily (R=0.001)
                    # This is our actual 'Truth' anchor.
                    kf.update(z_course, kf.H_yaw, np.eye(1)*0.001, angle_indices=[0])
            
            # Store History
            yaw_raw_ned = yaw_rad
            # Added Yaw Rate (Index 7) to results
            hist_est.append([t] + kf.x.flatten().tolist() + [yaw_raw_ned, current_yaw_rate])
            hist_ins_yaw.append([t, yaw_std])

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
        # Update Header: Timestamp, E, N, ...
        f.write('Timestamp, E, N, V, Yaw, Bias, Yaw_Raw, Yaw_Rate\n')
        for row in hist_est:
            # Row has 8 elements now
            f.write(f"{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}, {row[3]:.4f}, {row[4]:.4f}, {row[5]:.4f}, {row[6]:.4f}, {row[7]:.4f}\n")
    print(f"Results saved to {res_file}")

    # 2. Plot
    # Normalize
    if len(hist_gps) > 0:
        n0, e0 = hist_gps[0, 1], hist_gps[0, 2]
        hist_gps[:, 1] -= n0
        hist_gps[:, 2] -= e0
        hist_est[:, 1] -= n0
        hist_est[:, 2] -= e0
        
    plt.figure(figsize=(12, 12))
    plt.plot(hist_gps[:, 2], hist_gps[:, 1], 'k.', alpha=0.3, label='GPS (Zeroed)')
    plt.plot(hist_est[:, 2], hist_est[:, 1], 'b-', lw=1, alpha=0.5, label='EKF Trajectory')
    
    # Add Heading Arrows (Subsampled)
    # Plot every Nth point to avoid clutter
    step = 500 
    if len(hist_est) > step:
        sub_est = hist_est[::step]
        
        # Extract components
        x = sub_est[:, 2] # East
        y = sub_est[:, 1] # North
        yaw_corr = sub_est[:, 4]
        yaw_raw = sub_est[:, 6]
        
        # Vector Length (visual)
        vec_len = 20.0
        
        # Corrected (Green)
        u_corr = vec_len * np.cos(yaw_corr)
        v_corr = vec_len * np.sin(yaw_corr)
        plt.quiver(x, y, u_corr, v_corr, color='g', angles='xy', scale_units='xy', scale=1, width=0.003, label='Corrected Yaw')
        
        # Raw (Red)
        u_raw = vec_len * np.cos(yaw_raw)
        v_raw = vec_len * np.sin(yaw_raw)
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
    raw_yaw_deg = np.degrees(hist_est[:, 6])
    
    ax1.plot(time_arr, ekf_yaw_deg, 'g-', lw=2, label='EKF Yaw (Corrected)')
    ax1.plot(time_arr, raw_yaw_deg, 'r--', alpha=0.6, label='Raw INS Yaw (NED)')
    
    # Course Over Ground (Blue) - Line Plot
    # Calculate for all points using a stride to filter noise/low-speed
    stride = 50 # Every 50 samples (~0.5s if 100Hz)
    if len(hist_est) > stride:
        t_sub = time_arr[::stride]
        x_sub = hist_est[::stride, 2] # East
        y_sub = hist_est[::stride, 1] # North
        
        dx = np.diff(x_sub)
        dy = np.diff(y_sub)
        dist = np.hypot(dx, dy)
        
        # Only calculate angles where moving > 0.5m over the stride
        valid_mask = dist > 0.5
        
        if np.any(valid_mask):
            cog_angles = np.arctan2(dy[valid_mask], dx[valid_mask])
            cog_deg = np.degrees(cog_angles)
            # Plot corresponding times (offset by 1 due to diff)
            ax1.plot(t_sub[:-1][valid_mask], cog_deg, 'b-', lw=1.5, alpha=0.7, label='Course Truth (Blue)')
    
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
