#!/usr/bin/env python3

import rospy
from vectornav.msg import Ins
from geometry_msgs.msg import TwistStamped, PointStamped
from std_msgs.msg import Float32

from mote_ros.msg import State  # Replace with the actual package name if different
import numpy as np
import utm
import math
from math import sqrt
import tf.transformations
from kalman_filter import KalmanFilter
import datetime
import rospkg
import os


class KalmanFilterNode:
    def __init__(self):
        self.kf = KalmanFilter()
        self.initialized = False

        self.state_pub = rospy.Publisher('kalman_state', State, queue_size=10)
        self.utm_pub = rospy.Publisher('utm_state', State, queue_size=10)
        
        rospy.Subscriber('/vectornav/INS', Ins, self.ins_callback)
        rospy.Subscriber('/vehicle/twist', TwistStamped, self.twist_callback)

        self.prev_yaw = None
        self.prev_time = None
        
        # dynamic path to results folder
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mote_ros')
        # Go up two levels from package (src/mote_ros) to repo root
        repo_root = os.path.abspath(os.path.join(package_path, '../../'))
        self.output_file_path = os.path.join(repo_root, 'results', 'kalman_filter_results.txt')
        os.makedirs(os.path.dirname(self.output_file_path), exist_ok=True)
        rospy.loginfo(f"Writing results to: {self.output_file_path}")
        with open(self.output_file_path, 'w') as file:
            # Header matching animate_trajectory.py expectation (Time, N, E, V, Yaw, Bias) + Raw Yaw
            file.write('Time, N, E, V, Yaw, Bias, RawYaw\n')
            file.flush()

        self.current_yaw_rate = 0.0
        self.start_time = rospy.Time.now().to_sec()
        
        # Buffer for GPS history (Course Over Ground logic)
        self.hist_gps = [] 
        
        # TUNING PARAMETERS (Stronger trust to bridge the gap)
        self.velocity_fusion_threshold = 0.1
        self.course_fusion_trust = 0.002 
        self.turn_fusion_threshold = 0.05 
        self.min_geometry_distance = 0.1
        
        # Absolute parameters
        self.kf.Q[4, 4] = 0.00001 # Stable bias for coasting

    def latlon_to_utm(self, lat, lon):
        utm_coords = utm.from_latlon(lat, lon)
        return utm_coords[0], utm_coords[1]  # return Easting, Northing (ENU standard)

    def publish_state(self):
        # Publish the Kalman coordinates
        state = self.kf.x
        state_msg = State()
        state_msg.x_position = state[0, 0]
        state_msg.y_position = state[1, 0]
        state_msg.velocity = state[2, 0]
        state_msg.yaw = state[3, 0]  
        state_msg.yaw_rate = state[4, 0]
        self.state_pub.publish(state_msg)

    def log_state(self, raw_yaw):
        # Log state to file for animation
        curr_time = self.prev_time if self.prev_time else rospy.Time.now().to_sec()
        state = self.kf.x.flatten()
        line = f"{curr_time}, {state[0]}, {state[1]}, {state[2]}, {state[3]}, {state[4]}, {raw_yaw}\n"
        with open(self.output_file_path, 'a') as file:
            file.write(line)
            file.flush()

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
        # Convert Heading (NED 0=N, CW -> ENU 0=E, CCW)
        yaw_rad = math.radians(msg.yaw)
        yaw_std = math.pi/2.0 - yaw_rad
        yaw_std = (yaw_std + math.pi) % (2.0 * math.pi) - math.pi
        
        t_msg = msg.header.stamp.to_sec()

        if not self.initialized:
            # Initialize with Easting, Northing
            e, n = self.latlon_to_utm(msg.latitude, msg.longitude)
            self.origin_e = e
            self.origin_n = n
            self.kf.x[0, 0] = 0.0 # Relative E
            self.kf.x[1, 0] = 0.0 # Relative N
            self.kf.x[2, 0], self.kf.x[3, 0], self.kf.x[4, 0] = 0.0, yaw_std, 0.0
            self.prev_time = t_msg
            self.initialized = True
            return

        # Update dt and predict
        if not self.update_dt_and_predict(t_msg):
            return # Skip if dt is not valid

        # 1. Prediction with IMU (already done by update_dt_and_predict)
        # self.kf.predict_ekf(omega_measured=self.current_yaw_rate) # This is now handled by update_dt_and_predict

        # 2. Update with GPS Position
        e, n = self.latlon_to_utm(msg.latitude, msg.longitude)
        
        # if self.origin_x is None: # This block is now handled in initialization
        #     self.origin_x = e
        #     self.origin_y = n
            
        rel_e = e - self.origin_e
        rel_n = n - self.origin_n
        
        z_pos = np.array([[rel_e], [rel_n]]) # Now E, N
        self.kf.update(z_pos, self.kf.H_pos, self.kf.R_pos)
        
        # 3. Handle Course Fusion (using GPS history)
        self.hist_gps.append((rel_e, rel_n)) # Stores E, N
        stride = 50
        is_fusing_course = False
        
        if len(self.hist_gps) > stride:
            curr_e, curr_n = self.hist_gps[-1]
            past_e, past_n = self.hist_gps[-stride]
            de, dn = curr_e - past_e, curr_n - past_n # Delta East, Delta North
            dist = math.sqrt(de**2 + dn**2)
            est_v = self.kf.x[2, 0]
            
            moved_enough = dist > self.min_geometry_distance
            is_fast = est_v > self.velocity_fusion_threshold
            is_turning = abs(self.current_yaw_rate) > self.turn_fusion_threshold
            
            if moved_enough and (is_fast or is_turning):
                gps_course = math.atan2(dn, de)
                is_fusing_course = True
                z_course = np.array([[gps_course]])
                self.kf.update(z_course, self.kf.H_yaw, np.eye(1) * self.course_fusion_trust, angle_indices=[0])

        # 4. Sensor Update (Coasting)
        z_yaw = np.array([[yaw_std]])
        r_yaw_active = 1.0 if is_fusing_course else 100.0
        self.kf.update(z_yaw, self.kf.H_yaw, np.eye(1) * r_yaw_active, angle_indices=[0])

        if len(self.hist_gps) > 200: self.hist_gps.pop(0)

        self.publish_state()
        self.log_state(yaw_std)
            
    def twist_callback(self, msg):
        if not self.initialized: return
        t_msg = msg.header.stamp.to_sec()
        self.update_dt_and_predict(t_msg)

        z_vel = np.array([[msg.twist.linear.x]])
        z_yawrate = -msg.twist.angular.z 
        self.current_yaw_rate = z_yawrate

        self.kf.update(z_vel, self.kf.H_vel, self.kf.R_vel)
        self.publish_state()


if __name__ == '__main__':
    rospy.init_node('kalman_filter_node')
    node = KalmanFilterNode()
    rospy.spin()
