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
from papillarray_ros_v2.msg import SensorState
from slip_manipulation.sensorised_gripper import SensorisedGripper
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
    def __init__(self, box_dim, arm, grasp_param, box_weight):
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
        self.ft_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft_callback)
        self.tac_sub = rospy.Subscriber('/hub_0/sensor_0', SensorState, self.tac_callback)
        # self.gripper_sub = rospy.Subscriber('/Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, self.gripper_callback)
        self.gripper = SensorisedGripper()
        # self.gripper_pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        
        self.arm = arm

        self.theta = 0
        self.tac_data = 0
        # self.Fz = 0
        self.fg = 9.8 * box_weight * 1.5  # scaling shenanigans
        self.box_angle = np.arctan(self.height_dim/self.base_dim)

        self.angle_err = 1e6 # set large number to initialise while loop
        self.threshold = 5e-1
        self.Kp = 2e-4#1e-4
        self.Kd = 5e-4
        self.Ki = 8e-5#3e-6
        
        self.z_offset = 0
        self.x_offset = 0
        
        self.temp_loop = np.arange(5)
        
        self.theta = 0
        self.dt = time.time()
        self.prev_err = 0
        self.intg_err = 0
        
        self.slip_type = 0
        self.pillars_in_contact = 0
        

    def plan_cartesian_path(self, curr_angle, goal_angle):
        # Always assuming we start from the long direction
        # z is up and down, x is the direction of "home", y is the side  

        waypoints = []
        wpose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        
        r = np.hypot(self.base_dim, self.height_dim)
        box_angle = math.degrees(np.arctan(self.height_dim/self.base_dim))  + curr_angle
        
            # change is always positive
            # cos is always positive -> change * cos is positive -> x is positive
            # sin is always negative -> change * sin is negative -> y is negative
        for theta in np.linspace(box_angle, box_angle + goal_angle, 2 * int(np.ceil(0.5*abs(goal_angle - curr_angle)))):
            wpose.position.z = -(r*math.sin(math.radians(theta))) + self.height_dim# - 0.03
            wpose.position.y = self.base_dim - r*math.cos(math.radians(theta))
            
            wpose_base = tf_transform_pose(self.listener, wpose, 'tool0', 'base_link').pose
            waypoints.append(copy.deepcopy(wpose_base))
     
        return waypoints

    def angle_callback(self, data):
        self.theta = np.radians(data.angle)
        
    def ft_callback(self, data):
        self.Fz = data.wrench.force.z
        
    def tac_callback(self, data):
        self.tac_data = data
        
    def force_pred(self):
        # force = 20/np.pi * np.arctan(-8 * (self.theta - np.pi/5)) - 6
        theta = np.pi/2 - self.box_angle - self.theta
        alpha = np.pi/2 - theta
        force = self.fg * np.sin(theta) * np.cos(alpha)/2
        return force

    def control_robot(self, goal_angle):
        waypoints = self.plan_cartesian_path(self.theta, goal_angle)
        self.time = time.time()
        while self.angle_err > 1 or waypoints == []:
            temp_waypoints = []
            
            
            print('z_offset: ', self.z_offset)
            # print('x_offset: ', self.x_offset)
            
            waypoints[0].position.z = waypoints[0].position.z + self.z_offset
            waypoints[0].position.x = waypoints[0].position.x + self.x_offset
            temp_waypoints.append(copy.deepcopy(waypoints[0]))
            waypoints.pop(0)
            # print(temp_waypoints)
            (plan, _) = self.arm.compute_cartesian_path(temp_waypoints, 0.01, 0.0)
            execute_time = time.time()
            self.arm.execute(plan, wait=True)
            execute_time = time.time() - execute_time
            self.slip_control()
            # if self.z_offset < 0.05:
            #     self.arm.execute(plan, wait=True)
            # else:
            #     rospy.logerr("nah")
            # execute_time = time.time() - execute_time
            # rospy.sleep(1.5)
            predicted_force = self.force_pred()
            measured_force = self.Fz

            err = predicted_force - measured_force
            # print('error: ', err)
            # print("pred:", predicted_force)
            # print("meas:", measured_force)
            # print("error:", err)
            self.dt = execute_time
            # print('dt: ', self.dt)
            deriv_err = (err - self.prev_err) / self.dt 
            self.intg_err += err * self.dt
            # print("error:", err)
            # print("deriv error:", deriv_err)
            # print("intg error:", self.intg_err)
            
            # Error is positive when the robot is pushing too much, therefore the robot should raise its arm to reduce the force
            if err > self.threshold:
                # pass
                self.z_offset += ((self.Kp * err)) + (self.Ki * self.intg_err)
                # self.z_offset = ((self.Kp * err) + (self.Kd * deriv_err) + (self.Ki * self.intg_err)) * scale * abs(math.cos(2 * self.theta))
                # self.x_offset = ((self.Kp * err) + (self.Kd * deriv_err) + (self.Ki * self.intg_err)) * scale * math.sin(self.theta) * -np.sign(self.grasp_param)
            # Error is negative when the robot is lifting too much, therefore the robot should lower its arm to increase the force
            elif err < -self.threshold:
                # pass
                self.z_offset += (self.Kp * err) + (self.Ki * self.intg_err)
                # self.z_offset = ((self.Kp * err) + (self.Kd * deriv_err) + (self.Ki * self.intg_err)) * scale * abs(math.cos(2 * self.theta))
                # self.x_offset = ((self.Kp * err) + (self.Kd * deriv_err) + (self.Ki * self.intg_err)) * scale * math.sin(self.theta) * np.sign(self.grasp_param)
            # Do nothing
            else: 
                pass
            
            # print("Z offset: ", self.z_offset)
            # print("X offset: ", self.x_offset)

            
            self.angle_err = goal_angle - np.degrees(self.theta)
            self.time = time.time()
            self.prev_err = err
            
            # print("Angle error: ", self.angle_err)
            
        print("Pivot complete!")
        
    def slip_control(self):
        stop_tightening = 1
        for pillar in self.tac_data.pillars:
            if abs(pillar.dX) > 2.5 or abs(pillar.dY) > 2.5 or abs(pillar.dZ) > 2.5:
                self.gripper.send_gripper_command(commandName = None, grip_width=self.gripper.grip_width - 1)
                stop_tightening = 0
                print("Loosening grip")
            if abs(pillar.dY) > 0.1 and np.sign(pillar.dY) == -1:
                self.slip_type += 1
            if pillar.in_contact:
                self.pillars_in_contact += 1
        print(self.slip_type)
        print(self.pillars_in_contact)
        if self.slip_type == self.pillars_in_contact:
            print('Translational Slip')
            if self.gripper.grip_width  < 140 and stop_tightening:
                self.gripper.send_gripper_command(commandName = None, grip_width=self.gripper.grip_width  + 1)
                print('Tightening grip')
            print(self.gripper.grip_width)
        
            
        self.slip_type = 0
        self.pillars_in_contact = 0
        
            

if __name__ == "__main__":
    pass
