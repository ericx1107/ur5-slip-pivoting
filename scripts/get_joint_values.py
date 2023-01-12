#! /usr/bin/env python

import rospy
import moveit_commander
import numpy as np
import open3d
from time import sleep
from std_msgs.msg import String
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from geometry_msgs.msg import PoseStamped


if __name__ == "__main__":
    # initialise publishers and subscribers
    sleep(0.1) # Allow controller to start up (called in bash script)
    rospy.init_node('test')


    arm = moveit_commander.MoveGroupCommander('manipulator')
    # self.arm.set_planner_id("") # /home/acrv/HRIGroupAdmin/example_ros_ws/src/universal_robot/ur5_moveit_config/config/ompl_planning.yaml

    robot = moveit_commander.RobotCommander()

    # print(self.robot.get_group_names())
    # print('\n\n\n')
    print(robot.get_current_state())


# wrist_2_joint: 
# -2.614993397389547
# 0
# 2.5571610927581787