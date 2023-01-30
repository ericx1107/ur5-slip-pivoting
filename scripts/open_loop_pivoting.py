#! /usr/bin/env python

import rospy
import numpy as np
import copy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
from slip_manipulation.ur5_moveit import UR5Moveit
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.sensorised_gripper import SensorisedGripper
from slip_manipulation.arc_trajectory import ArcTrajectory

class OpenLoopPivoting():
    def __init__(self, box_dim, grasp_param):
        self.ur5 = UR5Moveit()
        
        self.grasp_param = grasp_param
        
        self.markers = BoxMarkers(box_dim, self.grasp_param)
        self.gripper = SensorisedGripper()
        self.arc = ArcTrajectory(box_dim, self.ur5.arm, self.grasp_param)
        
        # self.pos_grasp_sub = rospy.Subscriber('/slip_manipulation/grasp_pose', PoseStamped, self.callback)
        self.grasp_pub = rospy.Publisher('pregrasp_pose', PoseStamped, queue_size=1)

        self.pregrasp_offset = 0.05
        
        self.ee_offset = 0.16

        self.grasp_goal = PoseStamped()

    # def callback(self, data):
    #     self.grasp_goal = data
        # self.grasp_goal.pose.position.z += self.ee_offset


if __name__ == "__main__":
    rospy.init_node('open_loop_pivoting')
    box_dim = [0.18, 0.11, 0.04]
    
    while True:
        # get grasp position from user
        grasp_param = raw_input("Enter grasp location parameter (-1 to 1)\n")
        # check validity
        while True:
                try:
                    grasp_param = float(grasp_param)
                except ValueError:
                    print("Enter a number!\n")
                    continue

                if not -1 <= grasp_param <= 1:
                    print("Enter a number between -1 and 1!\n")
                    continue
                elif grasp_param == 0:
                    grasp_param += 0.0001
                
                # passed all checks
                break
        
        # initialise class object
        demo = OpenLoopPivoting(box_dim, grasp_param)
        # wait for some topics to publish
        # rospy.sleep(10)
        
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
        
        # set gripper orientation constraint
        gripper_constraints = Constraints()
        gripper_constraints.name = 'gripper_constraint'
            
        ori_constraint = OrientationConstraint()
        ori_constraint.link_name = 'wrist_3_link'
        ori_constraint.orientation = demo.ur5.arm.get_current_pose().pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 5*np.pi/180
        ori_constraint.absolute_y_axis_tolerance = 5*np.pi/180
        ori_constraint.absolute_z_axis_tolerance = 5*np.pi/180
        ori_constraint.weight = 1
        
        gripper_constraints.orientation_constraints.append(ori_constraint)

        demo.ur5.arm.set_path_constraints(gripper_constraints)
        
        demo.ur5.move_to_cartesian_goal(demo.grasp_goal.pose)

        # close gripper
        raw_input("press enter to close gripper")
        demo.gripper.send_gripper_command(commandName=None, grip_width=130) # 124 for long


        cartesian_plan= demo.arc.plan_cartesian_path()

        raw_input("check rviz before execute")
        demo.ur5.arm.execute(cartesian_plan, wait=True)

        # open gripper
        raw_input("press enter to open gripper")
        demo.gripper.send_gripper_command(commandName=None, grip_width=0)
        
        # move up
        end_goal = demo.ur5.arm.get_current_pose()
        end_goal.pose.position.z += 0.20
        demo.ur5.move_to_cartesian_goal(end_goal.pose)
        
        del demo
