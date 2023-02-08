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

        self.angle_sub = rospy.Subscriber('/slip_manipulation/rotation_angle', AngleStamped, self.angle_callback)

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
        for theta in np.linspace(box_angle, box_angle + goal_angle, 10):
            wpose.position.z = -(r*math.sin(math.radians(theta))) + self.height_dim
            wpose.position.y = self.base_dim - r*math.cos(math.radians(theta))
            
            wpose_base = tf_transform_pose(self.listener, wpose, 'tool0', 'base_link').pose
            waypoints.append(copy.deepcopy(wpose_base))
     
        return waypoints

    def angle_callback(self, data):
        self.theta = np.radians(data.angle)
        
    def force_pred(self):
        temp_angle = []
        for i in self.temp_loop:
            angle_stamped = rospy.wait_for_message('/slip_manipulation/rotation_angle', AngleStamped, timeout=rospy.Duration(10))
            temp_angle.append(angle_stamped.angle)
        self.theta = np.radians(np.average(temp_angle))
        force = 34.5/np.pi * np.arctan(-8 * (self.theta - np.pi/5)) - 10

        # theta = np.pi/2 - self.box_angle - self.theta
        # alpha = np.pi/2 - theta
        # force = self.fg * np.sin(theta) * np.cos(alpha)/2
        return force

    def control_robot(self, goal_angle):
        waypoints = self.plan_cartesian_path(self.theta, goal_angle)
        self.time = time.time()
        while self.angle_err > 2:
            temp_waypoints = []
            loop_length = np.arange(len(waypoints)) if len(waypoints) < 3 else np.arange(3)
            for i in loop_length:
                waypoints[0].position.z = waypoints[0].position.z + self.z_offset
                waypoints[0].position.x = waypoints[0].position.x + self.x_offset
                temp_waypoints.append(copy.deepcopy(waypoints[0]))
                waypoints.pop(0)
            (plan, _) = self.arm.compute_cartesian_path(temp_waypoints, 0.01, 0.0)
            self.arm.execute(plan, wait=True)
            predicted_force = self.force_pred()
            
            temp_force = []
            for i in self.temp_loop:
                wrench_stamped = rospy.wait_for_message('/robotiq_ft_wrench', WrenchStamped, timeout=rospy.Duration(5)).wrench.force.z
                temp_force.append(wrench_stamped)
            measured_force = np.average(temp_force)
            err = predicted_force - measured_force
            # print("pred:", predicted_force)
            # print("meas:", measured_force)
            # print("error:", err)
            self.dt = time.time() - self.time
            deriv_err = (err - self.prev_err) / self.dt 
            
            # Error is positive when the robot is pushing too much, therefore the robot should raise its arm to reduce the force
            if err > self.threshold:
                self.z_offset = ((self.Kp * err) + (self.Kd * deriv_err)) * math.sin(self.theta)
                self.x_offset = ((self.Kp * err) + (self.Kd * deriv_err)) * math.cos(self.theta) * np.sign(self.grasp_param)
            # Error is negative when the robot is lifting too much, therefore the robot should lower its arm to increase the force
            elif err < -self.threshold:
                self.z_offset = ((self.Kp * err) + (self.Kd * deriv_err)) * math.sin(self.theta)
                self.x_offset = ((self.Kp * err) + (self.Kd * deriv_err)) * math.cos(self.theta) * -np.sign(self.grasp_param)
            # Do nothing
            else: 
                self.z_offset = 0
                self.x_offset = 0
            # print("Z offset: ", self.z_offset)
            # print("X offset: ", self.x_offset)
            
            # plan = self.move_control_robot(self.z_offset, self.x_offset)
            
            self.angle_err = goal_angle - np.degrees(self.theta)
            self.time = time.time()
            self.prev_err = err
            
            # print("Angle error: ", self.angle_err)
            
            # for waypoint in waypoints:
            #     waypoint.position.z += self.z_offset
            #     waypoint.position.x += self.x_offset
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
        


if __name__ == "__main__":
    pass
    # tutorial = MoveGroupInteface()

    # cartesian_plan= tutorial.prepare()

    # raw_input("check rviz before execute")
    # tutorial.execute_plan(cartesian_plan)

