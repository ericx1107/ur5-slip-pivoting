#! /usr/bin/env python

import rospy
import numpy as np
import yaml
import tf
import tf2_ros
import argparse
import os
from datetime import tzinfo, timedelta, datetime
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
from pathlib import Path

class OptitrackTransform():
    '''publishes the transform from the OptiTrack world origin to a link on the robot.
    Requires a Base rigid body with the centre at the robot base.
    For the UR5 this can be done by placing markers onthe screw positions around the base.
    '''
    def __init__(self, save_transform=False, saved_transform_path=''):
        saved_transform_path = Path(saved_transform_path)
        # init subscribers and publishers as required
        # self.optitrack_base_sub = rospy.Subscriber("vrpn_client_node/Base/pose", PoseStamped)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        # self.tf_broadcaster = tf.TransformBroadcaster()
        
        # check for the base pose
        try:
            rospy.loginfo("waiting for message for base pose in OptiTrack frame")
            base_pose = rospy.wait_for_message("vrpn_client_node/Base/pose", PoseStamped, timeout=20.0)
            # convert pose message to a transform message
            # make a transform from the robot base_link to the optitrack origin "world"
            bTw_msg = TransformStamped()
            bTw_msg.header = base_pose.header
            bTw_msg.header.frame_id = "base_link"   #"base_link_inertia"
            bTw_msg.child_frame_id = "world"
            
            # build the homogeneous transformation matrix
            wtb = [base_pose.pose.position.x, 
                   base_pose.pose.position.y, 
                   base_pose.pose.position.z]
            wqb = [base_pose.pose.orientation.x, 
                   base_pose.pose.orientation.y, 
                   base_pose.pose.orientation.z, 
                   base_pose.pose.orientation.w]
            wRb = R.from_quat(wqb).as_dcm() #.as_matrix()       scipy versions!!!
            wTb = np.eye(4)
            wTb[:-1, :-1] = wRb
            wTb[:-1, -1] = wtb
            
            # do manual rotations to match the axes in Motive to in ROS
            manual_rot = np.eye(4)
            # lower case for extrinsic rotations
            ############ tune manually according to Base frame orientation in Motive ##############
            manual_rot[:-1, :-1] = R.from_euler('xyz', [0, -90, -90], degrees=True).as_dcm()
            ###########################
            
            wTb = np.matmul(wTb, manual_rot)    # rotation in the new frame after moving w->b, reversed order
            
            # invert the base position in the optitrack world frame to get the transform
            bTw = np.linalg.inv(wTb)
            bRw = bTw[:-1, :-1]
            # bqw = R.from_matrix(bRw).as_quat(scalar_first=False)    # also scipy versions
            bqw = R.from_dcm(bRw).as_quat()
            btw = bTw[:-1, -1]
            
            # fill the rest of the message
            bTw_msg.transform.translation = Vector3(*btw)
            bTw_msg.transform.rotation = Quaternion(*bqw)
            
            self.bTw_msg = bTw_msg
            
            if save_transform:
                class AEST(tzinfo):
                    def utcoffset(self, dt):
                        return timedelta(hours=10)
                    def tzname(self, dt):
                        return "AEST"
                    def dst(self, dt):
                        return timedelta(hours=11)
                aest = AEST()
                t = datetime.now(aest)
                dt_string = t.strftime("%Y_%m_%d___%H_%M_%S")
                filename = "base_transform_" + dt_string
                with open(str(saved_transform_path / filename), 'w+') as f:
                    yaml.dump(bTw_msg, f)
            
        except rospy.ROSException as e:
            rospy.loginfo("timeout waiting for base pose")
            # if the base pose is not detected, fall back to reading the transform from a file
            # look for a yaml file and load (maybe later make it load the most recent file)
            # for f in os.listdir(str(saved_transform_path)):
            #     if f.endswith(".yaml"):
            #         filename = f
            yaml_files = [file for file in saved_transform_path.iterdir() if file.suffix == '.yaml']
            filename = yaml_files[0]
            # load the YAML file
            with open(str(saved_transform_path / filename), 'r') as f:
                # yaml reads the file into a PoseStamped message
                self.bTw_msg = yaml.load(f, Loader=yaml.Loader)
        
        
    def publish_optitrack_origin_transform(self):
        self.tf_broadcaster.sendTransform(self.bTw_msg)
        rospy.loginfo("Publishing OptiTrack transform")
        
        while not rospy.is_shutdown():
            rospy.spin()
    
if __name__ == "__main__":
    rospy.init_node('optitrack_transform_publisher')
    # add input command if want to save the base transform into a file
    parser = argparse.ArgumentParser()
    parser.add_argument("--save_transform", action="store_true", 
                        help="whether to save the transform of the robot base into a file")
    args, _ = parser.parse_known_args()
    
    tpath = "/home/acrv/trajectory_ws/src/ur5_vistac_common/config"
    o = OptitrackTransform(save_transform=args.save_transform, saved_transform_path=tpath)
    o.publish_optitrack_origin_transform()
    