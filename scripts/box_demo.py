#! /usr/bin/env python

import rospy
import moveit_commander
import numpy as np
import open3d
import copy
from time import sleep
from std_msgs.msg import String
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import Constraints, JointConstraint
from slip_manipulation.ur5_moveit import UR5Moveit
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.sensorised_gripper import SensorisedGripper
from slip_manipulation.arc import Arc_Moving

class BoxDemo():
    def __init__(self, box_dim, grasp_param):
        self.ur5 = UR5Moveit()
        
        self.grasp_param = grasp_param
        
        self.markers = BoxMarkers(box_dim, self.grasp_param)
        self.gripper = SensorisedGripper()
        self.arc = Arc_Moving( self.markers.box_length, self.markers.box_height, self.ur5.arm)
        
        self.pos_grasp_sub = rospy.Subscriber('/slip_manipulation/grasp_pose', PoseStamped, self.callback)
        self.grasp_pub = rospy.Publisher('test_grasp', PoseStamped, queue_size=1)

        self.pregrasp_offset = 0.05
        
        self.ee_offset = 0.18

        self.grasp_goal = PoseStamped()

    def callback(self, data):
        self.grasp_goal = data
        self.grasp_goal.pose.position.z += self.ee_offset


if True:
    rospy.init_node('box_demo')
    box_dim = [0.18, 0.11, 0.04]
    
    # get grasp position from user
    grasp_param = raw_input("Enter grasp location parameter (-1 to 1)\n")
    # check validity
    while True:
            try:
                grasp_param = int(grasp_param)
            except ValueError:
                print("Enter a number!\n")
                continue

            if not -1 <= grasp_param <= 1:
                print("Enter a number between -1 and 1!\n")
                continue
            
            # passed all checks
            break
    
    # initialise class object
    demo = BoxDemo(box_dim, grasp_param)
    # wait for some topics to publish
    rospy.sleep(10)
    
    # get grasp goal while blocking
    demo.grasp_goal = rospy.wait_for_message('/slip_manipulation/grasp_pose', PoseStamped)
    demo.grasp_goal.pose.position.z += demo.ee_offset
    
    # move to goal
    pregrasp_pose = copy.deepcopy(demo.grasp_goal)
    pregrasp_pose.pose.position.z += demo.pregrasp_offset
    
    print("planning")
    demo.grasp_pub.publish(pregrasp_pose)
    
    # demo.markers.shutdown_detector()
    
    # demo.ur5.move_to_ee_goal(pregrasp_pose)
    demo.ur5.move_to_cartesian_goal(pregrasp_pose.pose)
    
    demo.ur5.move_to_cartesian_goal(demo.grasp_goal.pose)

    # close gripper
    raw_input("press enter to close gripper")
    demo.gripper.send_gripper_command(commandName=None, grip_width=123)


    cartesian_plan= demo.arc.plan_cartesian_path()

    raw_input("check rviz before execute")
    demo.ur5.arm.execute(cartesian_plan, wait=True)

    # open gripper
    raw_input("press enter to open gripper")
    demo.gripper.send_gripper_command(commandName=None, grip_width=0)


# ur5.arm.set_pose_reference_frame('base_link')

# input_pose = markers.marker_list["1"]["pose"]entation.y, 
                                                                            # self.box_pose.orientation.z, 
                                                                            # self.box_pose.orientation.w])
# print(type(input_pose))
# output_pose_stamped = ur5.tf_transform_pose(input_pose, 'camera_link', 'base_link')


# while not rospy.is_shutdown():
#     ur5.display_pose(output_pose_stamped)



'''ee_link = ur5.arm.get_end_effector_link()
start_pose = ur5.arm.get_current_pose(ee_link).pose
print(start_pose)
ur5 = UR5Moveit()
# goal_pose = [-0.150002384447, 0.0959219176177, 0.74666077793, 
# 0.658653290385, -0.611800540121, -0.269746822291, 0.345126924532]
# ur5.arm.set_pose_target(goal_pose)
start_pose.position.x += 0.1
goal_pose = [start_pose]
print(goal_pose)
plan, _ = ur5.arm.compute_cartesian_path(goal_pose, 0.01, 0.0)
# print(plan)
# plan = ur5.arm.plan()
raw_input('Press any button to execute')
ur5.arm.execute(plan)'''