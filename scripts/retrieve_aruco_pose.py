#! /usr/bin/env python

import rospy
from aruco_msgs.msg import MarkerArray
from collections import defaultdict
from geometry_msgs.msg import PoseStamped

class Markers():
    def __init__(self):
        rospy.init_node("marker_poses")

        self.markers_sub = rospy.Subscriber('/aruco_marker_publisher/markers', 
            MarkerArray, self.marker_callback)

        self.markers_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)

        self.marker_list = defaultdict(lambda: {"pose": 0})
    
    def marker_callback(self, data):
        for marker in data.markers:
            self.marker_list[str(marker.id)]["pose"] = marker.pose.pose
            
            # construct stamped message
            stamped = PoseStamped()
            stamped.header = marker.header
            stamped.pose = marker.pose.pose
            self.markers_pub.publish(stamped)

if __name__ == "__main__":
    marker = Markers()

    rospy.spin()