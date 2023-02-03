#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
import copy
import rospy
import math
import numpy as np
import tf2_ros
from slip_manipulation.get_tf_helper import *
from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped
from slip_manipulation.msg import AngleStamped

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
    def __init__(self, box_dim, arm, grasp_param, box_weight):
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

        self.angle_sub = rospy.Subscriber('/slip_manipulation/rotation_angle', AngleStamped, self.angle_callback)

        self.theta = 0
        self.Fz = 0
        self.fg = 9.8 * box_weight
        self.phi = np.arctan(self.height_dim/self.base_dim)

        self.angle_err = 1e6 # set large number to initialise while loop
        self.threshold = 1e-1
        self.Kp = 5e-3

        self.z_offset = 0
        self.x_offset = 0

    def plan_cartesian_path(self, curr_angle, goal_angle):
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
        box_angle = math.degrees(np.arctan(self.height_dim/self.base_dim))
        # axis_angle = self.arm.get_current_joint_values()[0]
     
        
            # change is always positive
            # cos is always positive -> change * cos is positive -> x is positive
            # sin is always negative -> change * sin is negative -> y is negative
        for theta in np.linspace(curr_angle + box_angle, goal_angle + box_angle, math.ceil(goal_angle - curr_angle)):
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
        return plan, waypoints

    def angle_callback(self, data):
        self.theta = np.radians(data.angle)
        
    def force_pred(self):
        temp_angle = []
        for i in range(5):
            angle_stamped = rospy.wait_for_message('/slip_manipulation/rotation_angle', AngleStamped, timeout=rospy.Duration(10))
            temp_angle.append(angle_stamped.angle)
        self.theta = np.radians(np.average(temp_angle))
        if 0 <= self.theta and self.theta < self.phi:
            force = -1 * self.fg * np.log(self.theta + (1 - self.phi))
        elif self.phi <= self.theta and self.theta <= np.pi/2:
            force = -0.25 * self.fg * (self.theta - self.phi)**3 * (self.theta - np.pi/2) * (self.theta + 11)
        return force

    def control_robot(self, goal_angle):
        # _, waypoints = self.plan_cartesian_path(self.theta, goal_angle)
        while self.angle_err > 2:
            # temp_waypoints = []
            # loop_length = len(waypoints) if len(waypoints) < 5 else 5
            # for i in range(loop_length):
            #     waypoints[0].position.z = waypoints[0].position.z + self.z_offset
            #     waypoints[0].position.x = waypoints[0].position.x + self.x_offset
            #     temp_waypoints.append(copy.deepcopy(waypoints[0]))
            #     waypoints.pop(0)
            # (plan, _) = self.arm.compute_cartesian_path(temp_waypoints, 0.01, 0.0)

            predicted_force = 7
            temp_force = []
            for i in range(6):
                wrench_stamped = rospy.wait_for_message('/robotiq_ft_wrench', WrenchStamped, timeout=rospy.Duration(5)).wrench.force.z
                temp_force.append(wrench_stamped)
            measured_force = np.average(temp_force)

            err = predicted_force - measured_force
            print("pred:", predicted_force)
            print("meas:", measured_force)
            print("error:", err)
            if self.angle_err >= self.theta and self.angle_err <= 90:
                self.z_offset = self.Kp * err
                self.x_offset = -0.25 * self.Kp * err
            elif self.angle_err < self.theta and self.angle_err >= 0:
                self.z_offset = 0.25 * self.Kp_z * err
                self.x_offset = self.Kp * err
            else: 
                self.z_offset = 0
                self.x_offset = 0
            print("Z offset: ", self.z_offset)
            
            plan = self.move_control_robot(self.z_offset, self.x_offset)
            self.arm.execute(plan, wait=True)
            
            self.angle_err = goal_angle - self.theta
            # upper = 9
            # lower = 0
            # print("meas:", measured_force)
            # if measured_force > upper:
            #     err = measured_force - upper
            #     self.x_offset = self.Kp * err
            #     print("error:", err)
            # elif measured_force < lower:
            #     err = lower - measured_force
            #     self.z_offset = self.Kp * err
            #     print("error:", err)
            # else:
            #     self.z_offset = 0
        print("Pivot complete!")
        
    def move_control_robot(self, z_offset, x_offset):
        waypoints = []
        wpose = self.arm.get_current_pose().pose
        while True:
            try:
                wpose = tf_transform_pose(self.tf_buffer, wpose, 'base_link', 'tool0').pose
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Waiting for box transform\n")
                
        temp = copy.deepcopy(wpose)
        wpose.position.y = temp.position.y - x_offset
        wpose.position.z = temp.position.z - z_offset
        wpose_base = tf_transform_pose(self.tf_buffer, wpose, 'tool0', 'base_link').pose
        waypoints.append(copy.deepcopy(wpose_base))
        (plan, _) = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan


if __name__ == "__main__":
    pass
    # tutorial = MoveGroupInteface()

    # cartesian_plan= tutorial.prepare()

    # raw_input("check rviz before execute")
    # tutorial.execute_plan(cartesian_plan)

