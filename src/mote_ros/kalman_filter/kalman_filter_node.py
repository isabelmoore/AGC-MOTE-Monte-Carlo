#!/usr/bin/env python3

import rospy
from vectornav.msg import Ins
from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

from mote_ros.msg import State 
import numpy as np
import utm
import math
import tf.transformations
from kalman_filter import KalmanFilter
import rospkg
import os


class KalmanFilterNode:
    def __init__(self):
        self.kf = KalmanFilter()
        self.initialized = False
        self.publish_seq = 0
        self.state_pub = rospy.Publisher('ekf_v8_final', State, queue_size=10)
        self.utm_pub = rospy.Publisher('utm_state', State, queue_size=10)
        
        # Phase 8: Viz Publishers
        self.viz_pub = rospy.Publisher('ekf_markers_v8', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('ekf_path_v8', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        rospy.Subscriber('/vectornav/INS', Ins, self.ins_callback)
        rospy.Subscriber('/vehicle/twist', TwistStamped, self.twist_callback)

        self.prev_yaw = None
        self.prev_time = None
        self.origin_e = None
        self.origin_n = None
        
        # dynamic path to results folder
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mote_ros')
        repo_root = os.path.abspath(os.path.join(package_path, '../../'))
        self.output_file_path = os.path.join(repo_root, 'results', 'kalman_filter_results.txt')
        os.makedirs(os.path.dirname(self.output_file_path), exist_ok=True)
        
        with open(self.output_file_path, 'w') as file:
            # Match 9-column format (T, E, N, V, Yaw, RateBias, HeadBias, Accel, RawYaw)
            file.write('Timestamp, E, N, V, Yaw, RateBias, HeadBias, Accel, RawYaw\n')
            file.flush()

        self.current_yaw_rate = 0.0
        self.current_linear_v = 0.0
        
        # Buffer for GPS history (Course Over Ground logic)
        self.hist_gps = [] 
        
        # --- PHASE 8 SYNC: Match Phase 7 Ultra-Smoothing ---
        self.velocity_fusion_threshold = 0.1
        self.course_fusion_trust = 0.2     # De-weight jumpy GPS heading
        self.turn_fusion_threshold = 0.05 
        self.min_geometry_distance = 0.15 
        
        self.kf.damping = 0.02 # Acceleration damping
        
        self.kf.R_pos = np.eye(2) * 15.0 # ULTRA smooth position
        self.kf.R_vel = np.eye(1) * 1.0
        self.kf.R_yaw = np.eye(1) * 0.1
        
        self.kf.Q[3, 3] = 0.0001    # Tightest heading (Heavy inertia)
        self.kf.Q[4, 4] = 0.000001  # Ultra-stable rate bias
        self.kf.Q[5, 5] = 0.0000001 # Ultra-stable heading bias
        self.kf.Q[6, 6] = 0.01      # Reduced accel noise
        # --------------------------------------------------
        
        rospy.loginfo("KalmanFilterNode Phase 8 Initialized (Ultra-Smoothing + Jitter Markers)")

    def latlon_to_utm(self, lat, lon):
        utm_coords = utm.from_latlon(lat, lon)
        return utm_coords[0], utm_coords[1] 

    def ned_to_enu(self, yaw_ned_rad):
        # NED (0=N, 90=E) -> ENU (0=E, 90=N)
        # yaw_enu = 90 - yaw_ned
        enu = (math.pi/2.0 - yaw_ned_rad)
        return (enu + math.pi) % (2.0 * math.pi) - math.pi

    def publish_state(self, raw_yaw_enu, gps_course=None):
        ti = rospy.Time.now()
        
        state = self.kf.x.flatten()
        state_msg = State()
        state_msg.message_seq = self.publish_seq  # Track sequence
        self.publish_seq += 1
        state_msg.x_position = state[0]
        state_msg.y_position = state[1]
        state_msg.velocity = state[2]
        
        # Convert internal radians to degree for ROS communication
        state_msg.yaw = math.degrees(state[3])
        state_msg.yaw_rate = math.degrees(state[4]) 
        state_msg.rate_bias = math.degrees(state[4]) # redundant but explicit
        state_msg.head_bias = math.degrees(state[5])
        state_msg.acceleration = state[6]
        ti_sec = ti.to_sec()
        crs_str = f"{gps_course:.1f}" if gps_course is not None else "nan"
        rospy.loginfo(f"PUB[{self.publish_seq}]: T={ti_sec % 1000:.1f} | EKF={state_msg.yaw:.1f} | Raw={math.degrees(raw_yaw_enu):.1f} | Bias={state_msg.head_bias:.1f} | Crs={crs_str} | V={state_msg.velocity:.2f}")
        self.state_pub.publish(state_msg)
        
        # 2. Path Viz
        pose = PoseStamped()
        pose.header.stamp = ti
        pose.header.frame_id = "map"
        pose.pose.position.x = state[0]
        pose.pose.position.y = state[1]
        q = tf.transformations.quaternion_from_euler(0, 0, state[3])
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > 500: self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)
        
        # 3. Markers (Arrows)
        ma = MarkerArray()
        
        def make_arrow(id, yaw, color, ns, scale=5.0):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = ti
            m.ns = ns
            m.id = id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = state[0]
            m.pose.position.y = state[1]
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x = scale
            m.scale.y = 0.5
            m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = color
            return m
            
        # Green: EKF
        ma.markers.append(make_arrow(0, state[3], (0, 1, 0, 1), "ekf_yaw"))
        # Red: Raw INS
        ma.markers.append(make_arrow(1, raw_yaw_enu, (1, 0, 0, 0.6), "raw_yaw"))
        # Blue: Course
        if gps_course is not None:
            ma.markers.append(make_arrow(2, gps_course, (0, 0, 1, 0.8), "course_yaw", scale=6.0))
        
        self.viz_pub.publish(ma)

    def log_state(self, raw_yaw_ned):
        curr_time = self.prev_time if self.prev_time else rospy.Time.now().to_sec()
        state = self.kf.x.flatten()
        # Log format: T, E, N, V, Yaw, RateBias, HeadBias, Accel, RawYaw(NED)
        line = f"{curr_time:.4f}, {state[0]:.4f}, {state[1]:.4f}, {state[2]:.4f}, {state[3]:.4f}, {state[4]:.6f}, {state[5]:.8f}, {state[6]:.4f}, {raw_yaw_ned:.4f}\n"
        with open(self.output_file_path, 'a') as file:
            file.write(line)

    def update_dt_and_predict(self, current_time):
        if self.prev_time is None:
            self.prev_time = current_time
            return False
        
        dt = current_time - self.prev_time
        if dt > 0:
            self.kf.dt = dt
            self.kf.predict_ekf(omega_measured=self.current_yaw_rate)
            self.prev_time = current_time
            return True
        return False

    def ins_callback(self, msg):
        # 1. Coordinate Conversion
        yaw_ned_rad = math.radians(msg.yaw)
        yaw_std = self.ned_to_enu(yaw_ned_rad)
        
        t_msg = msg.header.stamp.to_sec()

        if not self.initialized:
            e, n = self.latlon_to_utm(msg.latitude, msg.longitude)
            self.origin_e, self.origin_n = e, n
            # [E, N, V, Yaw, RateBias, HeadBias, Accel]
            self.kf.x = np.array([[0.0], [0.0], [0.0], [yaw_std], [0.0], [0.0], [0.0]])
            self.kf.P[5, 5] = 0.001 # Tighten bias uncertainty at start
            self.prev_time = t_msg
            self.initialized = True
            rospy.loginfo(f"EKF INITIALIZED: Yaw={math.degrees(yaw_std):.1f} deg")
            return

        self.update_dt_and_predict(t_msg)

        # 2. Update Position
        e, n = self.latlon_to_utm(msg.latitude, msg.longitude)
        rel_e, rel_n = e - self.origin_e, n - self.origin_n
        z_pos = np.array([[rel_e], [rel_n]]) 
        self.kf.update(z_pos, self.kf.H_pos, self.kf.R_pos)
        
        # 3. Handle Course Fusion
        self.hist_gps.append((rel_e, rel_n)) 
        stride = 50
        is_fusing_course = False
        gps_course = None
        
        if len(self.hist_gps) > stride:
            curr_e, curr_n = self.hist_gps[-1]
            past_e, past_n = self.hist_gps[-stride]
            de, dn = curr_e - past_e, curr_n - past_n 
            dist = math.hypot(de, dn)
            est_v = self.kf.x[2, 0]
            
            if dist > self.min_geometry_distance and (abs(est_v) > self.velocity_fusion_threshold or abs(self.current_yaw_rate) > self.turn_fusion_threshold):
                gps_course = math.atan2(dn, de)
                if est_v < -0.1:
                    gps_course = (gps_course + math.pi + math.pi) % (2.0 * math.pi) - math.pi
                
                is_fusing_course = True
                z_course = np.array([[gps_course]])
                self.kf.update(z_course, self.kf.H_yaw, np.eye(1) * self.course_fusion_trust, angle_indices=[0])

        # 4. Sensor Update
        z_yaw = np.array([[yaw_std]])
        if is_fusing_course:
            r_yaw_active = 100.0 
        else:
            is_stopped = abs(self.current_linear_v) < 0.02
            r_yaw_active = 200.0 if is_stopped else 20.0
            
        self.kf.update(z_yaw, self.kf.H_sensor, np.eye(1) * r_yaw_active, angle_indices=[0])

        if len(self.hist_gps) > 200: self.hist_gps.pop(0)

        # DEBUG PRINT: Comprehensive State View
        state = self.kf.x.flatten()
        ekf_deg = math.degrees(state[3])
        raw_deg = math.degrees(yaw_std)
        bias_deg = math.degrees(state[5])
        crs_deg = math.degrees(gps_course) if gps_course is not None else float('nan')
        
        # rospy.loginfo(f"PRINT:T={t_msg % 1000:.1f} | EKF={ekf_deg:.1f} | Raw={raw_deg:.1f} | Bias={bias_deg:.1f} | Crs={crs_deg:.1f} | V={state[2]:.2f}")

        self.last_raw_yaw_enu = yaw_std 
        self.publish_state(raw_yaw_enu=yaw_std, gps_course=gps_course)
        self.log_state(msg.yaw) # Use original degrees for file log
            
    def twist_callback(self, msg):
        if not self.initialized: return
        t_msg = msg.header.stamp.to_sec()
        self.update_dt_and_predict(t_msg)

        v_msg = msg.twist.linear.x
        self.current_linear_v = v_msg
        self.current_yaw_rate = msg.twist.angular.z 

        z_vel = np.array([[v_msg]])
        self.kf.update(z_vel, self.kf.H_vel, self.kf.R_vel)
        
        # --- PHASE 8 SYNC: Match Zero-Velocity Logic ---
        if abs(v_msg) < 0.01:
            self.kf.x[2, 0] = 0.0 
            self.kf.x[6, 0] = 0.0 
            
        # ry = self.last_raw_yaw_enu if hasattr(self, 'last_raw_yaw_enu') else self.kf.x[3,0]
        # self.publish_state(raw_yaw_enu=ry) 


if __name__ == '__main__':
    rospy.init_node('kalman_filter_node')
    node = KalmanFilterNode()
    rospy.spin()
