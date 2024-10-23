#! /usr/bin/env python

import rospy
import numpy as np
import copy
import subprocess
import time
import matplotlib as mpl
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, WrenchStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
from slip_manipulation.ur5_moveit import UR5Moveit
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.sensorised_gripper import SensorisedGripper
from slip_manipulation.arc_trajectory import ArcTrajectory
from slip_manipulation.simple_arc_trajectory import SimpleArc
from robotiq_ft_sensor.srv import sensor_accessor
from papillarray_ros_v2.srv import BiasRequest


if __name__ == "__main__":
    rospy.init_node('touch_object')
    
    g = SensorisedGripper()
    
    # dims: 4.4cm across orthogonal to holes, 3.8cm parallel to holes, 9.6 long
    
    width = 0.038
    g.touch_object(obj_width=width)