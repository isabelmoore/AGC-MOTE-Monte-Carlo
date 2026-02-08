#!/usr/bin/env python3

import rospy
import tf
import tf.transformations
import utm
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Point
from mote_ros.msg import State
from vectornav.msg import Ins
from std_msgs.msg import ColorRGBA, Float32MultiArray
import math

class VisualizationNode:
    def __init__(self):
        rospy.init_node('visualization_node')
        
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.yaw_data_pub = rospy.Publisher('yaw_comparison_data', Float32MultiArray, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber('ekf_v8_final', State, self.kalman_callback, queue_size=100)  # ← INCREASED
        rospy.Subscriber('/vectornav/INS', Ins, self.ins_callback)
        
        self.origin_e = None
        self.origin_n = None
        
        self.current_raw_x = None
        self.current_raw_y = None
        self.current_raw_yaw = 0.0
        
        # Buffers for history
        self.hist_pos = [] 
        self.trail_points = []
        self.last_time = rospy.Time.now()
        
        self.viz_counter = 0  # ← ADD THIS
        
    def ned_to_enu(self, yaw_ned_rad):
        enu = (math.pi/2.0 - yaw_ned_rad)
        return (enu + math.pi) % (2.0 * math.pi) - math.pi

    def ins_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude
        utm_e, utm_n, _, _ = utm.from_latlon(lat, lon)
        
        if self.origin_e is None:
            self.origin_n = utm_n
            self.origin_e = utm_e
            
        self.current_raw_x = utm_e - self.origin_e
        self.current_raw_y = utm_n - self.origin_n
        
        yaw_ned_rad = math.radians(msg.yaw)
        self.current_raw_yaw = self.ned_to_enu(yaw_ned_rad)

    def kalman_callback(self, msg):
        now = rospy.Time.now()
        
        if now < self.last_time:
            self.trail_points = []
            self.hist_pos = []
        self.last_time = now

        if self.current_raw_x is None:
            return
            
        x = self.current_raw_x
        y = self.current_raw_y
        
        yaw_deg = msg.yaw
        rospy.loginfo(f"FINAL_IN[{msg.message_seq}]: {msg.yaw:.1f} deg")
        yaw_rad = math.radians(yaw_deg)
        
        # Update Trail
        self.hist_pos.append((x, y))
        self.trail_points.append((x, y))
        if len(self.hist_pos) > 100: self.hist_pos.pop(0)

        # TF - Keep this, it's cheap
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
        self.tf_broadcaster.sendTransform((x, y, 0), quat, now, "base_link", "map")
        
        # ===== RATE LIMIT VISUALIZATION =====
        self.viz_counter += 1
        if self.viz_counter % 5 != 0:  # Only visualize every 5th message
            return
        # ====================================
        
        marker_array = MarkerArray()

        # 1. Path Trail
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = now
        path_marker.ns = "trail"
        path_marker.id = 10
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.3
        path_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        
        for p in self.trail_points:
            path_marker.points.append(Point(p[0], p[1], 0.1))
        marker_array.markers.append(path_marker)
        
        # 2. Raw Yaw Arrow
        raw_marker = Marker()
        raw_marker.header.frame_id = "map"
        raw_marker.header.stamp = now
        raw_marker.ns = "yaw_comparison"
        raw_marker.id = 1
        raw_marker.type = Marker.ARROW
        raw_marker.pose.position.x = x
        raw_marker.pose.position.y = y
        raw_marker.pose.position.z = 2.0
        q_raw = tf.transformations.quaternion_from_euler(0, 0, self.current_raw_yaw)
        raw_marker.pose.orientation = Quaternion(*q_raw)
        raw_marker.scale.x, raw_marker.scale.y, raw_marker.scale.z = 4.0, 0.1, 0.1
        raw_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        marker_array.markers.append(raw_marker)

        # 3. Filtered Yaw Arrow
        filt_marker = Marker()
        filt_marker.header.frame_id = "map"
        filt_marker.header.stamp = now
        filt_marker.ns = "yaw_comparison"
        filt_marker.id = 2
        filt_marker.type = Marker.ARROW
        filt_marker.pose.position.x = x
        filt_marker.pose.position.y = y
        filt_marker.pose.position.z = 2.3
        q_filt = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
        filt_marker.pose.orientation = Quaternion(*q_filt)
        filt_marker.scale.x, filt_marker.scale.y, filt_marker.scale.z = 4.0, 0.15, 0.15
        filt_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        marker_array.markers.append(filt_marker)

        # 4. Course Truth
        if len(self.hist_pos) > 30:
            past_e, past_n = self.hist_pos[-30]
            de, dn = x - past_e, y - past_n
            if math.hypot(de, dn) > 0.2:
                cog_marker = Marker()
                cog_marker.header.frame_id = "map"
                cog_marker.header.stamp = now
                cog_marker.ns = "yaw_comparison"
                cog_marker.id = 3
                cog_marker.type = Marker.ARROW
                cog_marker.pose.position.x = x
                cog_marker.pose.position.y = y
                cog_marker.pose.position.z = 2.6
                cog_angle = math.atan2(dn, de)
                q_cog = tf.transformations.quaternion_from_euler(0, 0, cog_angle)
                cog_marker.pose.orientation = Quaternion(*q_cog)
                cog_marker.scale.x, cog_marker.scale.y, cog_marker.scale.z = 4.0, 0.1, 0.1
                cog_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
                marker_array.markers.append(cog_marker)
                
                yaw_msg = Float32MultiArray()
                yaw_msg.data = [
                    math.degrees(self.current_raw_yaw), 
                    yaw_deg, 
                    math.degrees(cog_angle)
                ]
                self.yaw_data_pub.publish(yaw_msg)
        
        # REMOVE TERRAIN AND SUN - They're static and expensive
        
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    node = VisualizationNode()
    rospy.spin()