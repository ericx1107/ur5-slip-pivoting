#! /usr/bin/env python

import rospy
from aruco_msgs.msg import MarkerArray
from collections import defaultdict

class Markers():
    def __init__(self):
        rospy.init_node("marker_poses")

        self.markers_sub = rospy.Subscriber('/aruco_marker_publisher/markers', 
            MarkerArray, self.marker_callback)

        self.marker_list = defaultdict(lambda: {"position": [], "orientation": []})
    
    def marker_callback(self, data):
        for marker in data.markers:
            self.marker_list[str(marker.id)]["position"] = marker.pose.pose.position
            self.marker_list[str(marker.id)]["orientation"] = marker.pose.pose.orientation
        print(self.marker_list)

if __name__ == "__main__":
    marker = Markers()

    rospy.spin()