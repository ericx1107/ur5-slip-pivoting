#! /usr/bin/env python

import rospy
import numpy as np
import copy
import subprocess
import time
import tf
from std_msgs.msg import Float32
from slip_manipulation.msg import AngleStamped
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, WrenchStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
from slip_manipulation.ur5_moveit import UR5Moveit
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.sensorised_gripper import SensorisedGripper
from slip_manipulation.arc_trajectory import ArcTrajectory
from slip_manipulation.get_tf_helper import *
from robotiq_ft_sensor.srv import sensor_accessor
from papillarray_ros_v2.srv import BiasRequest



class OpenLoopPivoting():
    def __init__(self, box_dim, box_weight):
        self.ur5 = UR5Moveit()
        
        self.grasp_param = -0.00001
        
        self.markers = BoxMarkers(box_dim, self.grasp_param)
        self.gripper = SensorisedGripper()
        self.arc = ArcTrajectory(box_dim, self.ur5, self.gripper, self.grasp_param, box_weight)
        
        # self.pos_grasp_sub = rospy.Subscriber('/slip_manipulation/grasp_pose', PoseStamped, self.callback)
        self.grasp_pub = rospy.Publisher('pregrasp_pose', PoseStamped, queue_size=1)
        
        self.work_pub = rospy.Publisher('/slip_manipulation/work', Float32, queue_size=1)
        
        self.ft_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft_callback)

        self.pregrasp_offset = 0.05
        
        self.ee_offset = 0.18

        self.grasp_goal = PoseStamped()
        
        self.original = 0
        self.prev_angle = 0
        
        self.grasped = False
        
        self.rotating = False
        
        self.rotation_finish = False
        
        self.work = 0
        self.transalational_work = 0
        self.rotational_work = 0
        
        self.listener = tf.TransformListener()
        
        rospy.wait_for_service('/robotiq_ft_sensor_acc', timeout=rospy.Duration(10))
        robotiq_sensor_srv = rospy.ServiceProxy('/robotiq_ft_sensor_acc', sensor_accessor)
        
        
        
        try:
            resp1 = robotiq_sensor_srv(command_id = 8)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            
        rospy.wait_for_service('/hub_0/send_bias_request', timeout=rospy.Duration(10))
        tactile_sensor_srv = rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)
        try:
            resp2 = tactile_sensor_srv()
            print('response:' , resp2)
            # print('zeroed tactile sensor')
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))        

    # def callback(self, data):
    #     self.grasp_goal = data
        # self.grasp_goal.pose.position.z += self.ee_offset
        
    def ft_callback(self, data):
        self.Fz = data.wrench.force.z
        self.Fy = data.wrench.force.y
        self.Tx = data.wrench.torque.x
        
        trans = patient_lookup_tf(self.markers.tf_buffer, 'tool0')
        
        # eef_pose = Pose()
        # eef_pose.position.x = trans.transform.translation.x
        # eef_pose.position.y = trans.transform.translation.y
        # eef_pose.position.z = trans.transform.translation.z
        # eef_pose.orientation = trans.transform.rotation
        
        if self.grasped and (not self.rotating):
            # print("Fz is ", self.Fz)
            # print("Distance change is ", abs(trans.transform.translation.z - self.original.pose.position.z))
            self.transalational_work += abs(self.Fz * abs(trans.transform.translation.z - self.original.pose.position.z))
            self.transalational_work += abs(self.Fy * abs(trans.transform.translation.y - self.original.pose.position.y))
            
            self.original.pose.position.z = trans.transform.translation.z 
            self.original.pose.position.y = trans.transform.translation.y
        elif self.grasped and self.rotating:
            self.transalational_work += abs(self.Fz * abs(trans.transform.translation.y - self.original.pose.position.y))
            self.transalational_work += abs(self.Fy * abs(trans.transform.translation.z - self.original.pose.position.z))
            
            self.original.pose.position.z = trans.transform.translation.z 
            self.original.pose.position.y = trans.transform.translation.y
        
        
        
        if self.rotating and (not self.rotation_finish):
            
            orientation_rpy = np.array(tf.transformations.euler_from_quaternion([trans.transform.rotation.x, 
                                                                                        trans.transform.rotation.y, 
                                                                                        trans.transform.rotation.z, 
                                                                                        trans.transform.rotation.w]))

            # describes the axis orientations in terms of the base_link axes
            gripper_rot_mat = tf.transformations.euler_matrix(*orientation_rpy, axes='sxyz')[:3, :3]

            gripper_axis = gripper_rot_mat[:, 2]
            base_axis = [0,0,-1]
            
            angle = np.arccos(np.dot(gripper_axis, base_axis) / (np.linalg.norm(gripper_axis) * np.linalg.norm(base_axis)))
            angle_diff = abs(angle - self.prev_angle) 
            self.prev_angle = angle
            
            self.rotational_work += abs(self.Tx * angle_diff)
            
            # angle *= 180/np.pi
                
        # self.work = abs(self.ur5.arm.get_current_pose().pose.position.z * self.Fz)
        # self.work = abs(self.ur5.arm.get_current_pose().pose.position.y * self.Fy)
        
        

    # def angle_callback(self, data):
    #     self.theta = np.radians(data.angle)
        
    #     if self.grasped:
            

if __name__ == "__main__":
    rospy.init_node('open_loop_pivoting')
    # box_dim = [0.18, 0.11, 0.04]  # small box
    # box_dim = [0.23, 0.16, 0.05]  # square box
    box_dim = [0.28, 0.12, 0.05]  # long box
    
    # box_weight = 1.276    # small box
    # box_weight = 0.884    # square box
    # box_weight = 1.722    # long box
    box_weight = 0
    while not rospy.is_shutdown():
        # initialise class object
        demo = OpenLoopPivoting(box_dim, box_weight)
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
        
        demo.ur5.move_to_cartesian_goal(demo.grasp_goal.pose)

        # close gripper
        raw_input("press enter to close gripper")
        # demo.gripper.touch_object(box_dim, box_weight)
        demo.gripper.send_gripper_command(commandName='close')
        # 138 smallest box feb10(fri)
        # 103 square-ish box feb10(fri)
        # 107 long box feb13(mon)
        
        
        
        # lift object
        
        
        
        lift_offset = 0.12
        lift_pose = copy.deepcopy(demo.grasp_goal)
        lift_pose.pose.position.z += lift_offset
        
        demo.original = demo.ur5.arm.get_current_pose()
        demo.grasped = True
        demo.ur5.move_to_cartesian_goal(lift_pose.pose)
        
        
        # print('total work ', (demo.transalational_work + demo.rotational_work))
        # print('transalational work ', demo.transalational_work)
        # print('rotational work ',  demo.rotational_work)
        
        
        
        
        ############### rotate #####################
        # rospy.sleep(5)
        
        rpy = (np.pi/2, 0, 0)
        quat = tf.transformations.quaternion_from_euler(*rpy)
        init_rotated_pose = Pose(Point(0, demo.ee_offset, demo.ee_offset), Quaternion(*quat))
        rotated_pose = tf_transform_pose(demo.listener, init_rotated_pose, 'wrist_3_link', 'base_link').pose

        demo.rotating = True
        
        demo.ur5.move_to_cartesian_goal(rotated_pose)
        
        demo.rotation_finish = True
        
        print('rotated')
        
        
        #############################################
        
        # joints = demo.ur5.arm.get_current_joint_values()
        
        # joints[4] -= np.pi/2
        
        # demo.ur5.move_to_joints_goal(joints)
        
        # move down
        down_pose = demo.ur5.arm.get_current_pose()
        down_pose.pose.position.z += -lift_offset + 0.02

        # raw_input("press enter to move down")
        demo.ur5.move_to_cartesian_goal(down_pose.pose)

        # open gripper
        # raw_input("press enter to open gripper")
        demo.gripper.send_gripper_command(commandName=None, grip_width=0)
        
        print('total work ', (demo.transalational_work + demo.rotational_work))
        print('transalational work ', demo.transalational_work)
        print('rotational work ',  demo.rotational_work)
        demo.work_pub.publish((demo.transalational_work + demo.rotational_work))
        # move up
        end_goal = demo.ur5.arm.get_current_pose()
        end_goal.pose.position.z += 0.03
        demo.ur5.move_to_cartesian_goal(end_goal.pose)
        
        del demo
        
