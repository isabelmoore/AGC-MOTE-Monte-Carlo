#!/usr/bin/env python3.9

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_markers():
    rospy.init_node('marker_publisher')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "arrows"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0  # Arrow length
        marker.scale.y = 0.1  # Arrow width
        marker.scale.z = 0.1  # Arrow height

        # Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the start and end points of the arrow
        start_point = Point()
        start_point.x = 0
        start_point.y = 0
        start_point.z = 0

        end_point = Point()
        end_point.x = 1
        end_point.y = 0
        end_point.z = 0

        marker.points.append(start_point)
        marker.points.append(end_point)

        marker_pub.publish(marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_markers()
    except rospy.ROSInterruptException:
        pass

