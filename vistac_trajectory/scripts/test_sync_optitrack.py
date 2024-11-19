#! /usr/bin/env python

import rospy
import numpy as np
import copy
import time
import matplotlib as mpl
import matplotlib.pyplot as plt
import os
from datetime import datetime, timedelta
from geometry_msgs.msg import PoseStamped, WrenchStamped, Pose, Point, Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint
from ur5_vistac_common.ur5_moveit import UR5Moveit
from ur5_vistac_common.sensorised_gripper import SensorisedGripper
from vistac_trajectory.simple_arc_trajectory import SimpleArc
from robotiq_ft_sensor.srv import sensor_accessor
from papillarray_ros_v2.srv import BiasRequest

class FollowSimpleTrajectory():
    def __init__(self):
        self.ur5 = UR5Moveit()

        self.gripper = SensorisedGripper()
        self.arc = SimpleArc(self.ur5, self.gripper)
        
        self.grasp_pub = rospy.Publisher('pregrasp_pose', PoseStamped, queue_size=1)
        
        self.ft_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft_callback)
        self.ft_record = False
        self.Fy_array = []
        self.Fz_array = []
        self.t_array = []

        # ott stuff #####
        # init data arrays for the object poses and the robot eef pose
        self.object_poses = []
        self.robot_poses = []
        self.object_dist_from_orig = []
        self.robot_dist_from_orig = []
        self.object_t = []
        self.robot_t = []
        
        # init subscriber for vrpn object poses
        self.record = False
        self.init = True
        self.obj_start_t = 0
        self.rob_start_t = 0
        self.vrpn_sub = rospy.Subscriber('/vrpn_client_node/tool0/pose', PoseStamped, self.ott_callback)
        ################

        self.pregrasp_offset = 0.05
        
        self.ee_offset = 0.18

        self.grasp_goal = PoseStamped()
        
        rospy.wait_for_service('/robotiq_ft_sensor_acc', timeout=rospy.Duration(10))
        robotiq_sensor_srv = rospy.ServiceProxy('/robotiq_ft_sensor_acc', sensor_accessor)
        try:
            resp1 = robotiq_sensor_srv(command_id = 8)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
          
        # tactile sensor bias #######################################
        # rospy.wait_for_service('/hub_0/send_bias_request', timeout=rospy.Duration(10))
        # tactile_sensor_srv = rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)
        # try:
        #     resp2 = tactile_sensor_srv()
        #     print('response:' , resp2)
        #     # print('zeroed tactile sensor')
        # except rospy.ServiceException as exc:
        #     print("Service did not process request: " + str(exc))        
        # tactile sensor bias #######################################

    def ft_callback(self, data):
        self.Fz = data.wrench.force.z
        self.Fy = data.wrench.force.y
        self.Tx = data.wrench.torque.x
        
        if self.ft_record:
            self.Fz_array.append(data.wrench.force.z)
            self.Fy_array.append(data.wrench.force.y)
            self.t_array.append(data.header.stamp.to_nsec())

    def plot_ft(self):
        fig, ax = plt.subplots()
    
        ax.scatter(self.t_array, self.Fz_array, s=(mpl.rcParams['lines.markersize'] ** 2)/8)
        ax.set_xlabel('Time (Unix epoch)')
        ax.set_ylabel('Force in the z direction (N)')
        
        fig.savefig('/home/acrv/trajectory_ws/data/ft_' + str(time.time()) + '.png')

    def ott_callback(self, data):
        if not self.record:
            return
        
        robot_pose = self.ur5.arm.get_current_pose()
        
        obj_x = data.pose.position.x
        obj_y = data.pose.position.y
        obj_z = data.pose.position.z
        rob_x = robot_pose.pose.position.x
        rob_y = robot_pose.pose.position.y
        rob_z = robot_pose.pose.position.z
        
        obj_dist_from_orig = obj_z #np.sqrt(obj_x**2 + obj_y**2 + obj_z**2)
        rob_dist_from_orig = rob_z #np.sqrt(rob_x**2 + rob_y**2 + rob_z**2)
        
        
        if self.init:
            self.obj_start_t = data.header.stamp.to_nsec()
            self.rob_start_t = robot_pose.header.stamp.to_nsec()
            self.obj_rob_dist_diff = rob_dist_from_orig - obj_dist_from_orig
            self.init = False
            
        # self.object_poses.append(data.pose)
        self.object_t.append(data.header.stamp.to_nsec() - self.obj_start_t)
        # self.robot_poses.append(robot_pose.pose)
        # print(robot_pose.header)
        self.robot_t.append(robot_pose.header.stamp.to_nsec() - self.rob_start_t)
        
        self.object_dist_from_orig.append(obj_dist_from_orig)
        self.robot_dist_from_orig.append(rob_dist_from_orig - self.obj_rob_dist_diff)

if __name__ == "__main__":
    rospy.init_node('simple_arc')
    
    while True:
        
        # initialise class object
        demo = FollowSimpleTrajectory()
        # wait for some topics to publish
        # rospy.sleep(10)
        
        print(demo.ur5.arm.get_end_effector_link())
        
        start_quat = demo.ur5.arm.get_current_pose().pose.orientation
        
        # move the robot to a set pose
        # generate random points for the start and goal positions, with fixed orientation pointing down
        start_bounds = [[-0.60, -0.40, 0.25], [-0.40, -0.20, 0.50]]   # lower bound for xyz, upper bound for xyz
        goal_bounds = [[-0.60, 0.20, 0.25], [-0.40, 0.40, 0.50]]   # lower bound for xyz, upper bound for xyz
        s_point = (np.random.uniform(start_bounds[0][0], start_bounds[1][0]), 
                   np.random.uniform(start_bounds[0][1], start_bounds[1][1]),
                   np.random.uniform(start_bounds[0][2], start_bounds[1][2]))
        g_point = (np.random.uniform(goal_bounds[0][0], goal_bounds[1][0]), 
                   np.random.uniform(goal_bounds[0][1], goal_bounds[1][1]),
                   np.random.uniform(goal_bounds[0][2], goal_bounds[1][2]))
        start_pose = Pose(Point(*s_point), start_quat)
        goal_pose = Pose(Point(*g_point), start_quat)

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


        # move to the start pose (beginning of the arc)
        print("planning to random pose")
        
        plan, _ = demo.ur5.arm.compute_cartesian_path([start_pose], # waypoints to follow
										        0.01,       # eef_step  
										        0.0)        # jump_threshold  
        
        
        raw_input('\nCheck Rviz for cartesian plan of the trajectory, press enter to execute')
        
        demo.record = True
        rospy.sleep(1)
        demo.ur5.arm.execute(plan, wait=True)
        
        rospy.sleep(1)
        demo.record = False
        
        # print(demo.robot_t)
        
        # plot the two arrays against the time stamps
        fig = plt.figure(figsize=(6, 5))
        ax1 = fig.add_subplot(111)
        ax1.scatter(demo.object_t, demo.object_dist_from_orig, s=(mpl.rcParams['lines.markersize'] ** 2)/8, c='darkslateblue', label='mocap dist')
        ax1.scatter(demo.robot_t, demo.robot_dist_from_orig, s=(mpl.rcParams['lines.markersize'] ** 2)/8, c='gold', label='proprio dist')
        ax1.set_xlabel('Time')
        ax1.set_ylabel('displacement from origin (m)')
        ax1.legend()
    
    
        # ax2 = fig.add_subplot(122)
        # ax2.scatter(demo.robot_t, demo.robot_dist_from_orig, s=(mpl.rcParams['lines.markersize'] ** 2)/8)
        # ax2.set_xlabel('Time')
        # ax2.set_ylabel('eef distance from origin')
        
        plt.show()
        
        save_dir = "/home/acrv/trajectory_ws/data/tool0_ros_time_z/"
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        utc_offset_hours = -11
        utc_offset = timedelta(hours=utc_offset_hours)
        naive_datetime = datetime.now()
        timezone_adjusted_datetime = naive_datetime + utc_offset
        dt_string = timezone_adjusted_datetime.strftime("%Y_%m_%d__%H_%M_%S")
        
        np.save(save_dir + "obj_dist_" + dt_string, demo.object_dist_from_orig)
        np.save(save_dir + "obj_time_" + dt_string, demo.object_t)
        np.save(save_dir + "rob_dist_" + dt_string, demo.robot_dist_from_orig)
        np.save(save_dir + "rob_time_" + dt_string, demo.robot_t)
        
        fig.savefig(save_dir + "plot_" + dt_string + ".png")
        
        del demo
        
        break
