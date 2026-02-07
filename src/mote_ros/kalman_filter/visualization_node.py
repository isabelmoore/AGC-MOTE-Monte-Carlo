#!/usr/bin/env python3

import rospy
import tf
import tf.transformations
import utm
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Point
from mote_ros.msg import State
from vectornav.msg import Ins
from std_msgs.msg import ColorRGBA
import math

class VisualizationNode:
    def __init__(self):
        rospy.init_node('visualization_node')
        
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber('kalman_state', State, self.kalman_callback)
        rospy.Subscriber('/vectornav/INS', Ins, self.ins_callback)
        
        self.origin_x = None
        self.origin_y = None
        
        self.current_raw_x = None
        self.current_raw_y = None
        self.current_raw_yaw = 0.0
        
        # Buffers for history
        self.hist_pos = [] 
        self.trail_points = []
        self.last_time = rospy.Time.now()
        
    def ins_callback(self, msg):
        # 1. Position: Strictly from Raw Lat/Lon (as in the bag)
        lat, lon = msg.latitude, msg.longitude
        utm_e, utm_n, _, _ = utm.from_latlon(lat, lon)
        
        if self.origin_x is None:
            self.origin_x = utm_n
            self.origin_y = utm_e
            
        self.current_raw_x = utm_n - self.origin_x
        self.current_raw_y = utm_e - self.origin_y
        
        # 2. Raw Yaw for Red Arrow
        yaw_rad = math.radians(msg.yaw)
        self.current_raw_yaw = (math.pi/2.0 - yaw_rad + math.pi) % (2.0 * math.pi) - math.pi

    def kalman_callback(self, msg):
        now = rospy.Time.now()
        
        # Clear trail if time jump back detected (Bag Reset)
        if now < self.last_time:
            self.trail_points = []
            self.hist_pos = []
        self.last_time = now

        # We need the raw position to proceed
        if self.current_raw_x is None:
            return
            
        # COORDINATES: Always Raw GPS
        x = self.current_raw_x
        y = self.current_raw_y
        
        # HEADING: Use Corrected EKF Yaw
        yaw = msg.yaw
        
        # Update Trail - No limit now
        self.hist_pos.append((x, y))
        self.trail_points.append((x, y))
        if len(self.hist_pos) > 100: self.hist_pos.pop(0)

        # Publish TF for base_link (Jeep)
        # Position: Raw | Orientation: Corrected
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.tf_broadcaster.sendTransform((x, y, 0),
                                          quat,
                                          rospy.Time.now(),
                                          "base_link",
                                          "map")
        
        marker_array = MarkerArray()

        # 1. Path Trail (Red Line)
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
        
        # 2. Raw Yaw (Red Arrow)
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

        # 3. Filtered Yaw (Green Arrow)
        filt_marker = Marker()
        filt_marker.header.frame_id = "map"
        filt_marker.header.stamp = now
        filt_marker.ns = "yaw_comparison"
        filt_marker.id = 2
        filt_marker.type = Marker.ARROW
        filt_marker.pose.position.x = x
        filt_marker.pose.position.y = y
        filt_marker.pose.position.z = 2.3
        q_filt = tf.transformations.quaternion_from_euler(0, 0, yaw)
        filt_marker.pose.orientation = Quaternion(*q_filt)
        filt_marker.scale.x, filt_marker.scale.y, filt_marker.scale.z = 4.0, 0.15, 0.15
        filt_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        marker_array.markers.append(filt_marker)

        # 4. Course Truth (Blue Arrow) - Calculated from raw pos delta
        if len(self.hist_pos) > 50:
            past_x, past_y = self.hist_pos[0]
            dx, dy = x - past_x, y - past_y
            if math.hypot(dx, dy) > 0.1:
                cog_marker = Marker()
                cog_marker.header.frame_id = "map"
                cog_marker.header.stamp = now
                cog_marker.ns = "yaw_comparison"
                cog_marker.id = 3
                cog_marker.type = Marker.ARROW
                cog_marker.pose.position.x = x
                cog_marker.pose.position.y = y
                cog_marker.pose.position.z = 2.6
                cog_angle = math.atan2(dy, dx)
                q_cog = tf.transformations.quaternion_from_euler(0, 0, cog_angle)
                cog_marker.pose.orientation = Quaternion(*q_cog)
                cog_marker.scale.x, cog_marker.scale.y, cog_marker.scale.z = 4.0, 0.1, 0.1
                cog_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
                marker_array.markers.append(cog_marker)
        
        # 5. ENVIRONMENT markers
        terrain = Marker()
        terrain.header.frame_id = "map"; terrain.header.stamp = now; terrain.ns = "env"; terrain.id = 100
        terrain.type = Marker.CUBE; terrain.pose.position.z = -0.1
        terrain.scale.x, terrain.scale.y, terrain.scale.z = 1000.0, 1000.0, 0.1
        terrain.color = ColorRGBA(0.2, 0.4, 0.2, 1.0); marker_array.markers.append(terrain)

        sun = Marker()
        sun.header.frame_id = "map"; sun.header.stamp = now; sun.ns = "env"; sun.id = 101
        sun.type = Marker.SPHERE; sun.pose.position.x, sun.pose.position.y, sun.pose.position.z = 150, 150, 80
        sun.scale.x, sun.scale.y, sun.scale.z = 25, 25, 25
        sun.color = ColorRGBA(1.0, 0.9, 0.0, 1.0); marker_array.markers.append(sun)

        # 6. YAW BARS (Poor Man's Plotter)
        # We will draw 3 bars relative to the car to show the angles
        # Normalized angle display (0.0 to 1.0 height for 0 to 360 deg)
        # Red Bar (Raw)
        bar_r = Marker()
        bar_r.header.frame_id = "base_link"; bar_r.header.stamp = now; bar_r.ns = "plot"; bar_r.id = 200
        bar_r.type = Marker.CUBE; bar_r.pose.position.x, bar_r.pose.position.y = 0, 2.0
        h_r = (self.current_raw_yaw + math.pi) / (2*math.pi) * 5.0 # Height 0-5m
        bar_r.pose.position.z = h_r / 2.0
        bar_r.scale.x, bar_r.scale.y, bar_r.scale.z = 0.5, 0.5, h_r if h_r > 0.01 else 0.01
        bar_r.color = ColorRGBA(1,0,0,1); marker_array.markers.append(bar_r)

        # Green Bar (EKF)
        bar_g = Marker()
        bar_g.header.frame_id = "base_link"; bar_g.header.stamp = now; bar_g.ns = "plot"; bar_g.id = 201
        bar_g.type = Marker.CUBE; bar_g.pose.position.x, bar_g.pose.position.y = 1.0, 2.0
        h_g = (yaw + math.pi) / (2*math.pi) * 5.0 
        bar_g.pose.position.z = h_g / 2.0
        bar_g.scale.x, bar_g.scale.y, bar_g.scale.z = 0.5, 0.5, h_g if h_g > 0.01 else 0.01
        bar_g.color = ColorRGBA(0,1,0,1); marker_array.markers.append(bar_g)
        
        # Blue Bar (Truth) - Only if moving
        if len(self.hist_pos) > 50:
            past_x, past_y = self.hist_pos[0]
            dx, dy = x - past_x, y - past_y
            if math.hypot(dx, dy) > 0.1:
                cog_angle = math.atan2(dy, dx)
                bar_b = Marker()
                bar_b.header.frame_id = "base_link"; bar_b.header.stamp = now; bar_b.ns = "plot"; bar_b.id = 202
                bar_b.type = Marker.CUBE; bar_b.pose.position.x, bar_b.pose.position.y = 2.0, 2.0
                h_b = (cog_angle + math.pi) / (2*math.pi) * 5.0 
                bar_b.pose.position.z = h_b / 2.0
                bar_b.scale.x, bar_b.scale.y, bar_b.scale.z = 0.5, 0.5, h_b if h_b > 0.01 else 0.01
                bar_b.color = ColorRGBA(0,0,1,1); marker_array.markers.append(bar_b)

        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    node = VisualizationNode()
    rospy.spin()
