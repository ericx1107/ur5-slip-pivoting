#! /usr/bin/env python

import rospy
import actionlib
from armer_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from geometry_msgs.msg import PoseStamped

class UR5Armer():
    
    def __init__(self):

        # initialise moveit planning scene
        self.pose_cli = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
        
    def armer_move_to_pose_goal(self, goal_pose): 
        target = PoseStamped()

        target.pose = goal_pose
        target.header.frame_id = "base_link"
        target.header.stamp = rospy.Time.now()
        
        goal = MoveToPoseGoal()
        goal.pose_stamped = target
        self.pose_cli.send_goal(goal)
        result = self.pose_cli.wait_for_result()
        print(self.pose_cli.get_goal_status_text())
        print(self.pose_cli.get_result())
        print(self.pose_cli.get_state())
        