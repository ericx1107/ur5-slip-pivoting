#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math
import json
import numpy as np
import tf2_ros
from slip_manipulation.get_tf_helper import *

'''class MoveGroupInteface(object):
	def __init__(self):
		super(MoveGroupInteface, self).__init__()
		######################### setup ############################
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('ur_move_test_node', anonymous=True)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()  # Not used in this tutorial
		group_name = "manipulator"  # group_name can be find in ur5_moveit_config/config/ur5.srdf
		self.move_group_commander = moveit_commander.MoveGroupCommander(group_name)
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
		
		################ Getting Basic Information ######################
		self.planning_frame = self.move_group_commander.get_planning_frame()
		# print("============ Planning frame: %s" % self.planning_frame)
		self.eef_link = self.move_group_commander.get_end_effector_link()
		# print("============ End effector link: %s" % self.eef_link)
		self.group_names = self.robot.get_group_names()


	def prepare(self):
		# file = open('data.json')
		long = True
		box_height = 0.11 if long else 0.18
		box_length =0.18 if long else 0.11

		wpose = self.move_group_commander.get_current_pose().pose
		temp = copy.deepcopy(wpose)
		print("current: ", wpose.position.z)
		test = Arc_Moving(temp, box_length, box_height, self.move_group_commander)
		plan = test.plan_cartesian_path()
			

		return plan

	def execute_plan(self, plan):
		## Use execute if you would like the robot to follow
		## the plan that has already been computed:
		self.move_group_commander.execute(plan, wait=True)'''

class Arc_Moving():
    # pose: Current Pose of the end effector
    # box_length: box length that lays down on the ground
    # box_height: box height value
    # long: boolean for if we are making the long direction of the box lay down on the ground
    # arm: "the move_group_commander", used for cartesian path
    # shift: the shifted value along the x direction
    def __init__(self,  box_length, box_height, arm, shift=0):
        self.length = box_length-shift
        self.height = box_height
        # self.pose = 
        self.shift = shift
        self.arm = arm
        self.forward = True
        self.cof = 1 if self.forward else -1 

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def plan_cartesian_path(self):
        # Always assuming we start from the long direction
        # z is up and down, x is the direction of "home", y is the side  

        waypoints = []
        wpose = self.arm.get_current_pose().pose
        # while True:
            # try:
                # wpose = tf_transform_pose(self.tf_buffer, wpose, 'base_link', 'box_origin').pose
                # break
            # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # print("Waiting for box transform\n")
            # 
        temp = copy.deepcopy(wpose)
        print(temp)
        r = np.hypot(self.length, self.height)
        angle = math.degrees(np.arctan(self.height/self.length))
        axis_angle = self.arm.get_current_joint_values()[0]
     
        
            # change is always positive
            # cos is always positive -> change * cos is positive -> x is positive
            # sin is always negative -> change * sin is negative -> y is negative
        for theta in np.linspace(angle,90+angle,100):
            change =  (self.length - self.shift - r*math.cos(math.radians(theta)))
            wpose.position.z = r*math.sin(math.radians(theta)) + 0.15
            
            wpose.position.x = temp.position.x - (change ) * math.cos(axis_angle) 
            wpose.position.y = temp.position.y - (change ) * math.sin(axis_angle)
            
            # wpose = tf_transform_pose(self.tf_buffer, wpose, 'box_origin', 'base_link').pose
            waypoints.append(copy.deepcopy(wpose)) 
            print("z change is", wpose.position.z - temp.position.z)
            print("x change is", wpose.position.x - temp.position.x)
            print("y change is", wpose.position.y - temp.position.y)
            print("change" , change)
            print("--------------------------")
        
  
        
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

