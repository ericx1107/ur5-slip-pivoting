#! /usr/bin/env python

import rospy
import numpy as np
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.state_estimator import StateEstimator



if __name__ == "__main__":
    rospy.init_node("contact_estimation")

markers = BoxMarkers([0.18,0.11,0.04])
rospy.sleep(2)
l = 0.18
h = 0.11
r = np.hypot(l/2,h/2)
theta = np.arctan(h/l)
# print(theta * (180/np.pi))
x, y, yaw_arr = [], [], []
error = 0.05 * 2 * np.pi
if len(markers.marker_list.keys()) != 0:
    for i in range(10):
        # x.append(markers.marker_list["2"]["pose"].position.x)
        # y.append(markers.marker_list["2"]["pose"].position.y)
        orientation_q = markers.marker_list["4"]["pose"].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw_val = euler_from_quaternion(orientation_list)
        yaw_arr.append(yaw_val)
    # avg_x = np.average(x)
    # avg_y = np.average(y)
    avg_yaw = np.average(yaw_arr)
    # x_shift = -l/2 - avg_x
    # y_shift = -h/2 - avg_y
    yaw_shift_arr = [-np.pi - avg_yaw, -np.pi/2 - avg_yaw, 0 - avg_yaw, np.pi/2 - avg_yaw, np.pi - avg_yaw]
    abs_yaw_arr = np.abs(yaw_shift_arr)
    abs_yaw_shift = np.min(abs_yaw_arr)
    index = np.where(abs_yaw_arr == abs_yaw_shift)
    yaw_shift = yaw_shift_arr[index[0][0]]
    # print(x_shift)
    # print(markers.marker_list["1"]["pose"].position.x)
    
while not rospy.is_shutdown():
    if len(markers.marker_list.keys()) != 0:
        position = markers.marker_list["4"]["pose"].position
        orientation_q = markers.marker_list["4"]["pose"].orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        yaw_aligned = yaw + yaw_shift
        print('Roll', roll*180/np.pi)
        
        print('Pitch', pitch*180/np.pi)
        
        print('Yaw', yaw*180/np.pi)
        if (-error <= yaw_aligned <= error) or (np.pi - error <= yaw_aligned <= np.pi) or (-np.pi >= yaw_aligned >= -np.pi + error):
            print('Short Side')
        elif (-np.pi/2 - error <= yaw_aligned <= -np.pi/2 + error) or (np.pi/2 - error <= yaw_aligned <= np.pi/2 + error):
            print('Long Side')
        else:
            print('Pivoting')
        
        # print(distance)
        # print(position.y)
        # print(position.x)
        # x_aligned = x_shift + position.x
        # y_aligned = y_shift + position.y
        # yaw_aligned = yaw_shift + yaw
        # print(yaw)
        # distance = r*np.sin(theta+yaw_aligned)
        # if np.abs(y_aligned) > 1.05*(h/2) and -0.05 < np.sin(yaw_aligned) < 0.05:
        #     # print("No contact with long surface")
        #     print("No contact with short surface")
        # elif np.abs(y_aligned) > 1.05*(l/2) and (-0.985 > np.sin(yaw_aligned) or np.sin(yaw_aligned) > 0.985):
        #     # print("No contact with short surface")
        #     print("No contact with long surface")
        # elif np.abs(y_aligned) <= 1.05*(h/2) and -0.05 < np.sin(yaw_aligned) < 0.05:
        #     # print("Contact with long side")
        #     print("Contact with short side")
        # elif np.abs(y_aligned) > 1.05*(h/2) and np.abs(y_aligned) <= 1.05*(l/2) and (-0.95 > np.sin(yaw_aligned) or np.sin(yaw_aligned) > 0.95):
        #     # print("Contact with short side")
        #     print("Contact with long side")
        # else:
            
        #     pred_y = np.abs(x_aligned) * np.tan(theta+yaw_aligned)
        #     # print('Predicted y: ' + str(pred_y))
        #     # print('Measured y: ' + str(np.abs(position.y)))

        #     # print('Aligned x: ' + str(x_aligned))
        #     # print('Predicted y: ' + str(pred_y))
        #     # print('Measured y: ' + str(np.abs(position.y)))

        #     if pred_y <= 1.05 * np.abs(position.y):
        #         print("No contact with surface")
        #     else:
        #         print("Pivotting")
        