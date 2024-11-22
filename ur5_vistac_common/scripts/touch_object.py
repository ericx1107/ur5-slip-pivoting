#! /usr/bin/env python

import rospy
from ur5_vistac_common.sensorised_gripper import SensorisedGripper


if __name__ == "__main__":
    rospy.init_node('touch_object')
    
    g = SensorisedGripper()
    
    # initial 3d print block
    # dims: 4.4cm across orthogonal to holes, 3.8cm parallel to holes, 9.6 long
    # width = 0.038
    # roughly 103 touch
    # roughly 115 firm
    
    width = float(raw_input("Enter width of object in m to save time:"))
    print("\n")
    g.touch_object(obj_width=width)