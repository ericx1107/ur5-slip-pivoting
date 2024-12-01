#! /usr/bin/env python

import rospy
import numpy as np
import yaml
import tf
import tf2_ros
import argparse
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion

class OptitrackTransform():
    '''Catches the pose of the robot pose 
    '''
    def __init__(self, save_transform=False, saved_transform_path=''):
        # init subscribers and publishers as required
        # self.optitrack_base_sub = rospy.Subscriber("vrpn_client_node/Base/pose", PoseStamped)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # check for the base pose
        try:
            base_pose = rospy.wait_for_message("vrpn_client_node/Base/pose", PoseStamped, timeout=5.0)
            # convert pose message to a transform message
            # make a transform from the robot base_link to the optitrack origin "world"
            bTw_msg = TransformStamped()
            bTw_msg.header = base_pose.header
            bTw_msg.header.frame_id = "base_link"
            bTw_msg.child_frame_id = "world"
            
            # build the homogeneous transformation matrix
            wtb = [base_pose.position.x, base_pose.position.y, base_pose.position.z]
            wqb = [base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w]
            wRb = R.from_quat(wqb).as_matrix()
            wTb = np.eye(4)
            wTb[:-1, :-1] = wRb
            wTb[:-1, -1] = wtb
            # invert the base position in the optitrack world frame to get the transform
            bTw = np.linalg.inv(wTb)
            bRw = bTw[:-1, :-1]
            bqw = R.from_matrix(bRw).as_quat(scalar_first=False)
            btw = bTw[:-1, -1]
            
            # fill the rest of the message
            bTw_msg.transform.translation = Vector3(*btw)
            bTw_msg.transform.rotation = Quaternion(*bqw)
            
            self.bTw_msg = bTw_msg
            
            if save_transform:
                with open(saved_transform_path, 'w+') as f:
                    yaml.dump(bTw_msg, f)
            
        except rospy.ROSException:
            # if the base pose is not detected, fall back to reading the transform from a file
            # Read the YAML file
            with open(saved_transform_path, 'r') as f:
                # yaml reads the file into a PoseStamped message
                self.bTw_msg = yaml.load(f, Loader=yaml.Loader)
        
        
    def publish_optitrack_origin_transform(self):
        self.tf_broadcaster.sendTransform(self.bTw_msg)
        while not rospy.is_shutdown():
            rospy.spin()
    
if __name__ == "__main__":
    # add input command if want to save the base transform into a file
    parser = argparse.ArgumentParser()
    parser.add_argument("--save_transform", action="store_true", help="whether to save the transform of the robot base into a file")
    args = parser.parse_args()
    
    o = OptitrackTransform(save_transform=args.save_transform)
    o.publish_optitrack_origin_transform()
    