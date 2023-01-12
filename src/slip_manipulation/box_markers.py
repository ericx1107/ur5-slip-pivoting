#! /usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
from aruco_msgs.msg import MarkerArray, Marker
from collections import defaultdict
from geometry_msgs.msg import PoseStamped, TransformStamped

class BoxMarkers():
    def __init__(self):
        self.markers_sub = rospy.Subscriber('/aruco_marker_publisher/markers', 
            MarkerArray, self.marker_callback)

        self.markers_pose_pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)

        self.marker_list = defaultdict(lambda: {"pose": 0})

        self.tf_br = tf2_ros.TransformBroadcaster()
    
    def marker_callback(self, marker_array):
        for marker in marker_array.markers:
            self.marker_list[str(marker.id)]["pose"] = marker.pose.pose
            
            # construct stamped message
            stamped = PoseStamped()
            stamped.header = marker.header
            stamped.pose = marker.pose.pose

            # publish marker pose
            self.markers_pose_pub.publish(stamped)

            # publish tf's
            self.publish_marker_transform(stamped, marker.id)

    def publish_marker_transform(self, pose_stamped:type[PoseStamped], marker_id):
        t = TransformStamped()

        t.header = pose_stamped.header
        t.child_frame_id = 'marker_' + marker_id
        t.transform.translation.x = pose_stamped.pose.position.x
        t.transform.translation.y = pose_stamped.pose.position.y
        t.transform.translation.z = pose_stamped.pose.position.z
        t.transform.rotation.x = pose_stamped.pose.orientation.x
        t.transform.rotation.y = pose_stamped.pose.orientation.y
        t.transform.rotation.z = pose_stamped.pose.orientation.z
        t.transform.rotation.w = pose_stamped.pose.orientation.w

        self.tf_br.sendTransform(t)

    def publish_box_origin_transform(self, marker_id):
        if marker_id == 3 or marker_id == 4:
            z_offset = -0.114/2
        elif marker_id == 5 or marker_id == 6:
            z_offset = -0.18/2
        elif marker_id == 1 or marker_id == 2:
            z_offset = -0.04/2
            
        t = TransformStamped()

        t.header.stamp = rospy.Time(0)
        t.header.frame_id = 'marker_' + marker_id
        t.child_frame_id = 'box_origin'
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = z_offset
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_br.sendTransform(t)

if __name__ == "__main__":
    pass