#! /usr/bin/env python

import rospy
import numpy as np
import copy
import subprocess
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
from slip_manipulation.ur5_moveit import UR5Moveit
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.sensorised_gripper import SensorisedGripper
from slip_manipulation.arc_trajectory import ArcTrajectory
from robotiq_ft_sensor.srv import sensor_accessor
from papillarray_ros_v2.srv import BiasRequest

class OpenLoopPivoting():
    def __init__(self, box_dim, grasp_param, box_weight):
        self.ur5 = UR5Moveit()
        
        self.grasp_param = grasp_param
        
        self.markers = BoxMarkers(box_dim, self.grasp_param)
        self.gripper = SensorisedGripper()
        self.arc = ArcTrajectory(box_dim, self.ur5.arm, self.grasp_param, box_weight)
        
        # self.pos_grasp_sub = rospy.Subscriber('/slip_manipulation/grasp_pose', PoseStamped, self.callback)
        self.grasp_pub = rospy.Publisher('pregrasp_pose', PoseStamped, queue_size=1)

        self.pregrasp_offset = 0.05
        
        self.ee_offset = 0.18

        self.grasp_goal = PoseStamped()
        
        rospy.wait_for_service('/robotiq_ft_sensor_acc', timeout=rospy.Duration(10))
        robotiq_sensor_srv = rospy.ServiceProxy('/robotiq_ft_sensor_acc', sensor_accessor)
        try:
            resp1 = robotiq_sensor_srv(command_id = 8)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            
        # rospy.wait_for_service('/hub_0/send_bias_request', timeout=rospy.Duration(10))
        # tactile_sensor_srv = rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)
        # try:
        #     resp2 = tactile_sensor_srv()
        #     print('response:' , resp2)
        #     # print('zeroed tactile sensor')
        # except rospy.ServiceException as exc:
        #     print("Service did not process request: " + str(exc))        

    # def callback(self, data):
    #     self.grasp_goal = data
        # self.grasp_goal.pose.position.z += self.ee_offset


if __name__ == "__main__":
    rospy.init_node('open_loop_pivoting')
    # box_dim = [0.18, 0.11, 0.04]  # small box
    box_dim = [0.28, 0.12, 0.05]  # long box
    # box_dim = [0.23, 0.16, 0.05]  # square box
    # box_weight = 1.276    # small box
    # box_weight = 0.884    # square box
    box_weight = 1.722    # long box
    while True:
        # get grasp position from user
        grasp_param = raw_input("Enter grasp location parameter (-1 to 1)\n")
        # check validity
        while True:
                try:
                    grasp_param = float(grasp_param)
                except ValueError:
                    print("Enter a number!\n")
                    grasp_param = raw_input("Enter grasp location parameter (-1 to 1)\n")
                    continue

                if not -1 <= grasp_param <= 1:
                    print("Enter a number between -1 and 1!\n")
                    grasp_param = raw_input("Enter grasp location parameter (-1 to 1)\n")
                    continue
                elif grasp_param == 0:
                    grasp_param += 0.0001
                
                # passed all checks
                break
        
        # initialise class object
        demo = OpenLoopPivoting(box_dim, grasp_param, box_weight)
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
        demo.gripper.send_gripper_command(commandName=None, grip_width=107)
        # 138 smallest box feb10(fri)
        # 103 square-ish box feb10(fri)


        # cartesian_plan= demo.arc.plan_cartesian_path()

        # raw_input("check rviz before execute")
        # demo.ur5.arm.execute(cartesian_plan, wait=True)
        
        raw_input("press enter to move robot")
        demo.arc.control_robot(90)

        # open gripper
        raw_input("press enter to open gripper")
        demo.gripper.send_gripper_command(commandName=None, grip_width=0)
        
        # move up
        end_goal = demo.ur5.arm.get_current_pose()
        end_goal.pose.position.z += 0.20
        demo.ur5.move_to_cartesian_goal(end_goal.pose)
        
        del demo
        
        bashCommand = "rosnode kill /vision_estimator"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
