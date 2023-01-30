#! /usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Point, Quaternion
from slip_manipulation.get_tf_helper import *

class StateEstimator():
    def __init__(self, box_dim):
        self.box_dim = box_dim # lwh
        self.box_length = box_dim[0]
        self.box_width = box_dim[1]
        self.box_height = box_dim[2]
        
        # ros publishers and subscribers
        self.angle_pub = rospy.Publisher('/slip_manipulation/rotation_angle', Float32, queue_size=1)

        # tf things
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # initialise
        (self.upright_axis, self.upright_idx) = self.get_upright_axis()
        
    def estimate_contact_config(self, contact=True):

        trans = patient_lookup_box_tf(self.tf_buffer, loop=True)

        # get rotation matrix from euler angles
        orientation_rpy = np.array(tf_conversions.transformations.euler_from_quaternion([trans.transform.rotation.x, 
                                                                                        trans.transform.rotation.y, 
                                                                                        trans.transform.rotation.z, 
                                                                                        trans.transform.rotation.w]))

        # describes the axis orientations in terms of the base_link axes
        box_rot_mat = tf_conversions.transformations.euler_matrix(*orientation_rpy, axes='sxyz')[:3, :3]

        base_z_axis = [0, 0, 1]

        # find the box axis that is roughly pointing up (smallest angle between this and base_link z axis)
        angles = []

        for col in range(box_rot_mat.shape[1]):
            box_axis = box_rot_mat[:, col]
            angle = np.arccos(np.dot(box_axis, base_z_axis) / (np.linalg.norm(box_axis) * np.linalg.norm(base_z_axis)))
            angles.append(angle)

        # check for close to 0
        min_angle_idx = np.argmin(angles)
        min_angle_deg = angles[min_angle_idx] * 180/np.pi
        # check for close to 180
        max_angle_idx = np.argmax(angles)
        max_angle_deg = angles[max_angle_idx] * 180/np.pi
        
        if contact is None:
            return
        elif not contact:
            print("No contact")

        tol = 3 # degrees tolerance
        if np.isclose(min_angle_deg, 0, atol=tol) or np.isclose(min_angle_deg, -180, atol=tol) or np.isclose(max_angle_deg, 180, atol=tol):
            print("Contact through SURFACE")
        else:
            print("Contact through EDGE")

    def vision_estimate_contact(self):
        # check all eight vertices of the box
        coords = []
        coords.append((self.box_dim[0]/2, self.box_dim[1]/2, self.box_dim[2]/2))
        coords.append((self.box_dim[0]/2, self.box_dim[1]/2, -self.box_dim[2]/2))
        coords.append((self.box_dim[0]/2, -self.box_dim[1]/2, self.box_dim[2]/2))
        coords.append((self.box_dim[0]/2, -self.box_dim[1]/2, -self.box_dim[2]/2))
        coords.append((-self.box_dim[0]/2, self.box_dim[1]/2, self.box_dim[2]/2))
        coords.append((-self.box_dim[0]/2, self.box_dim[1]/2, -self.box_dim[2]/2))
        coords.append((-self.box_dim[0]/2, -self.box_dim[1]/2, self.box_dim[2]/2))
        coords.append((-self.box_dim[0]/2, -self.box_dim[1]/2, -self.box_dim[2]/2))

        z_height = []
        for coord in coords:
            vertex = Pose(Point(*coord), Quaternion(0, 0, 0, 1))
            base_vertex = tf_transform_pose(self.tf_buffer, vertex, 'box_origin', 'base_link', loop=False)
            if base_vertex is None:
                return
            z_height.append(base_vertex.pose.position.z)
        
        min_z_idx = np.argmin(z_height)
        min_z = z_height[min_z_idx]

        second_min_z = np.partition(z_height, 1)[1]

        if min_z > 0.02:
            print("No contact")
            return False

        elif min_z < 0.02 and not np.isclose(min_z, second_min_z, atol=0.01):
            print("Contact through VERTEX")
            return None

        else:
            return True

    def get_upright_axis(self):
        # check upright edge using transform from base_link
        trans = patient_lookup_box_tf(self.tf_buffer, loop=True)

        # get rotation matrix for condition check
        orientation_rpy = np.array(tf_conversions.transformations.euler_from_quaternion([trans.transform.rotation.x, 
                                                                                        trans.transform.rotation.y, 
                                                                                        trans.transform.rotation.z, 
                                                                                        trans.transform.rotation.w]))
        # orientation_rpy_for_print = orientation_rpy * 180/np.pi
        # print(orientation_rpy_for_print)
        
        tol = 0.2 # absolute tolerance, for value that should be between -1 to 1

        rot_mat = tf_conversions.transformations.euler_matrix(*orientation_rpy, axes='sxyz')[:3, :3]
        # print(rot_mat)
        new_z = rot_mat[:, 2]

        # need to find axis that is pointing in the direction of the z axis of the base_link frame (upright)
        # for rotation matrix: find column with [0; 0; 1] (roughly)
        # that column represents the axis that is transformed to point in the direction of the original z axis
        # if -1 that means it's pointing in opposite directino to original z (down)

        if all(np.isclose(new_z, [0, 0, 1], atol=tol)) or all(np.isclose(new_z, [0, 0, -1], atol=tol)):
            # print("z axis (blue) is vertical, long side")
            z_ori_coeff = np.sign(rot_mat[2, 2])    # is the new axis pointing in the same direction as the old z axis (up)?
                                                    # 1 if same direction, -1 if opposite
            
            return (np.array([0, 0, 1]) * z_ori_coeff, 2)

        elif all(np.isclose(new_z, [1, 0, 0], atol=tol)) or all(np.isclose(new_z, [-1, 0, 0], atol=tol)):
            # print("x axis (red) is vertical, short side")
            z_ori_coeff = np.sign(rot_mat[0, 2])    # is the new axis pointing in the same direction as the old z axis (up)?
                                                    # 1 if same direction, -1 if opposite

            return (np.array([1, 0, 0]) * z_ori_coeff, 0)

        elif all(np.isclose(new_z, [0, 1, 0], atol=tol)) or all(np.isclose(new_z, [0, -1, 0], atol=tol)):
            print("y axis (green) is vertical, no graspable edge!")
            # v_offset = self.box_dim[2]/2
            return 
        else:
            print("Uncaught case for object position. No side facing up found.")
            return

    def estimate_rotation_angle(self):
        try:
            trans = self.tf_buffer.lookup_transform('box_origin', 'base_link', rospy.Time(0), timeout=rospy.Duration(2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Waiting for box transform\n")
            return

        # get rotation matrix from euler angles
        orientation_rpy = np.array(tf_conversions.transformations.euler_from_quaternion([trans.transform.rotation.x, 
                                                                                        trans.transform.rotation.y, 
                                                                                        trans.transform.rotation.z, 
                                                                                        trans.transform.rotation.w]))

        # describes the axis orientations in terms of the base_link axes
        box_rot_mat = tf_conversions.transformations.euler_matrix(*orientation_rpy, axes='sxyz')[:3, :3]

        box_axis = box_rot_mat[:, self.upright_idx]
        angle = np.arccos(np.dot(box_axis, self.upright_axis) / (np.linalg.norm(box_axis) * np.linalg.norm(self.upright_axis)))
        angle *= 180/np.pi

        self.angle_pub.publish(angle)

if __name__ == "__main__":
    pass