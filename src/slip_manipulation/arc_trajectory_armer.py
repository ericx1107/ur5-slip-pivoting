#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
import copy
import rospy
import math
import numpy as np
import tf2_ros
import time
from geometry_msgs.msg import Pose, Point, Quaternion
from slip_manipulation.get_tf_helper import *
from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped
from slip_manipulation.msg import AngleStamped
import time

class ArcTrajectory():
    # pose: Current Pose of the end effector
    # base_dim: box length that lays down on the ground
    # height_dim: box height value
    # long: boolean for if we are making the long direction of the box lay down on the ground
    # arm: "the move_group_commander", used for cartesian path
    # shift: the shifted value along the x direction
    
    '''
    Always pivots towards the centre of box
    '''
    def __init__(self, box_dim, grasp_param, box_weight):
        self.long = rospy.wait_for_message('/slip_manipulation/is_long_edge', Bool, timeout=rospy.Duration(10))
        
        if self.long.data:
            # print("long is true")
            shift = (1 - abs(grasp_param)) * box_dim[0]/2
            self.base_dim = box_dim[0]-shift
            self.height_dim = box_dim[1]
            self.grasp_param = grasp_param
        else:
            # print("long is false")
            shift = (1 - abs(grasp_param)) * box_dim[1]/2
            self.base_dim = box_dim[1]-shift
            self.height_dim = box_dim[0]
            self.grasp_param = grasp_param
        
        self.listener = tf.TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.theta = 0
        # self.Fz = 0
        self.fg = 9.8 * box_weight
        self.box_angle = np.arctan(self.height_dim/self.base_dim)

        self.angle_err = 1e6 # set large number to initialise while loop
        self.threshold = 5e-1
        self.Kp = 1e-4
        self.Kd = 1e-4

        self.z_offset = 0
        self.x_offset = 0
        
        self.temp_loop = np.arange(5)
        
        self.theta = 0
        self.dt = time.time()
        self.prev_err = 0
        

    def plan_cartesian_path(self, curr_angle, goal_angle):
        # Always assuming we start from the long direction
        # z is up and down, x is the direction of "home", y is the side  

        waypoints = []
        wpose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        
        r = np.hypot(self.base_dim, self.height_dim)
        box_angle = math.degrees(np.arctan(self.height_dim/self.base_dim))
        
            # change is always positive
            # cos is always positive -> change * cos is positive -> x is positive
            # sin is always negative -> change * sin is negative -> y is negative
        for theta in np.linspace(box_angle, box_angle + goal_angle, 30):
            wpose.position.z = -(r*math.sin(math.radians(theta))) + self.height_dim
            wpose.position.y = self.base_dim - r*math.cos(math.radians(theta))
            
            wpose_base = tf_transform_pose(self.listener, wpose, 'tool0', 'base_link').pose
            waypoints.append(copy.deepcopy(wpose_base))
     
        return waypoints

    def force_pred(self):
        temp_angle = []
        for i in self.temp_loop:
            angle_stamped = rospy.wait_for_message('/slip_manipulation/rotation_angle', AngleStamped, timeout=rospy.Duration(10))
            temp_angle.append(angle_stamped.angle)
        self.theta = np.radians(np.average(temp_angle))
        force = 20/np.pi * np.arctan(-8 * (self.theta - np.pi/5)) - 6

        # theta = np.pi/2 - self.box_angle - self.theta
        # alpha = np.pi/2 - theta
        # force = self.fg * np.sin(theta) * np.cos(alpha)/2
        return force

if __name__ == "__main__":
    pass
    # tutorial = MoveGroupInteface()

    # cartesian_plan= tutorial.prepare()

    # raw_input("check rviz before execute")
    # tutorial.execute_plan(cartesian_plan)

