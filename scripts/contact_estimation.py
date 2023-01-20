#! /usr/bin/env python

import rospy
import moveit_commander
import numpy as np
import open3d
from time import sleep
from std_msgs.msg import String
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import Constraints, JointConstraint
from slip_manipulation.ur5_moveit import UR5Moveit
from slip_manipulation.box_markers import BoxMarkers
from tf.transformations import euler_from_quaternion

rospy.init_node("test")

markers = BoxMarkers()
rospy.sleep(2)
l = 0.18
h = 0.11
r = np.hypot(l/2,h/2)
theta = np.arctan(h/l)
# print(theta * (180/np.pi))
x = []
if len(markers.marker_list.keys()) != 0:
    for i in range(10):
        x.append(markers.marker_list["1"]["pose"].position.x)
    avg_x = np.average(x)
    x_shift = -l/2 - avg_x

    # print(x_shift)
    # print(markers.marker_list["1"]["pose"].position.x)
    
while not rospy.is_shutdown():
    if len(markers.marker_list.keys()) != 0:
        position = markers.marker_list["1"]["pose"].position
        orientation_q = markers.marker_list["1"]["pose"].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        # print(yaw)
        distance = r*np.sin(theta+yaw)
        # print(distance)
        # print(position.y)
        # print(position.x)
        
        if np.abs(position.y) > 1.05*(h/2) and -0.05 < np.sin(yaw) < 0.05:
            print("No contact with long surface")
        elif np.abs(position.y) > 1.05*(l/2) and (-0.985 > np.sin(yaw) or np.sin(yaw) > 0.985):
            print("No contact with short surface")
        elif np.abs(position.y) <= 1.05*(h/2) and -0.05 < np.sin(yaw) < 0.05:
            print("Contact with long side")
        elif np.abs(position.y) > 1.05*(h/2) and np.abs(position.y) <= 1.05*(l/2) and (-0.95 > np.sin(yaw) or np.sin(yaw) > 0.95):
            print("Contact with short side")
        else:
            x_aligned = x_shift + position.x
            pred_y = np.abs(x_aligned) * np.tan(theta+yaw)
            print('Aligned x: ' + str(x_aligned))
            print('Predicted y: ' + str(pred_y))
            print('Measured y: ' + str(np.abs(position.y)))

            if pred_y <= 1.05 * np.abs(position.y):
                print("No contact with surface")
            else:
                print("Pivotting")
            