#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
import copy
import rospy
import math
import numpy as np
import tf2_ros
from slip_manipulation.get_tf_helper import *
from std_msgs.msg import Bool

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
    def __init__(self, box_dim, arm, grasp_param):
        self.long = rospy.wait_for_message('/slip_manipulation/is_long_edge', Bool, timeout=rospy.Duration(10))
        
        if self.long.data:
            # print("long is true")
            shift = (1 - abs(grasp_param)) * box_dim[0]/2
            self.base_dim = box_dim[0]-shift
            self.height_dim = box_dim[1]
        else:
            # print("long is false")
            shift = (1 - abs(grasp_param)) * box_dim[1]/2
            self.base_dim = box_dim[1]-shift
            self.height_dim = box_dim[0]
            
        self.arm = arm

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def plan_cartesian_path(self):
        # Always assuming we start from the long direction
        # z is up and down, x is the direction of "home", y is the side  

        waypoints = []
        wpose = self.arm.get_current_pose().pose
        while True:
            try:
                wpose = tf_transform_pose(self.tf_buffer, wpose, 'base_link', 'tool0').pose
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Waiting for box transform\n")
            
        temp = copy.deepcopy(wpose)
        # print(temp)
        r = np.hypot(self.base_dim, self.height_dim)
        angle = math.degrees(np.arctan(self.height_dim/self.base_dim))
        # axis_angle = self.arm.get_current_joint_values()[0]
     
        
            # change is always positive
            # cos is always positive -> change * cos is positive -> x is positive
            # sin is always negative -> change * sin is negative -> y is negative
        for theta in np.linspace(angle,85+angle,100):
            wpose.position.z = -(r*math.sin(math.radians(theta))) + self.height_dim
            wpose.position.y = (temp.position.y + (self.base_dim - r*math.cos(math.radians(theta))))
            
            wpose_base = tf_transform_pose(self.tf_buffer, wpose, 'tool0', 'base_link').pose
            waypoints.append(copy.deepcopy(wpose_base)) 

        
  
        
        (plan, _) = self.arm.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step  
                                        0.0)         # jump_threshold  
     
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        print("=========== Planning completed, Cartesian path is saved=============")
        return plan


if __name__ == "__main__":
    pass
    # tutorial = MoveGroupInteface()

    # cartesian_plan= tutorial.prepare()

    # raw_input("check rviz before execute")
    # tutorial.execute_plan(cartesian_plan)

