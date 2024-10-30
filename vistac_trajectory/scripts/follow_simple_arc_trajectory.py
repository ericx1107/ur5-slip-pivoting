#! /usr/bin/env python

import rospy
import numpy as np
import copy
import time
import matplotlib as mpl
import matplotlib.pyplot as plt
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

if __name__ == "__main__":
    rospy.init_node('simple_arc')
    
    while True:
        
        # initialise class object
        demo = FollowSimpleTrajectory()
        # wait for some topics to publish
        # rospy.sleep(10)
        
        # generate random points for the start and goal positions, with fixed orientation pointing down
        # TODO add actual bounds
        start_bounds = [[0, 0, 0], [0, 0, 0]]   # lower bound for xyz, upper bound for xyz
        goal_bounds = [[0, 0, 0], [0, 0, 0]]   # lower bound for xyz, upper bound for xyz
        s_point = (np.random.uniform(start_bounds[0][0], start_bounds[1][0]), 
                   np.random.uniform(start_bounds[0][1], start_bounds[1][1]),
                   np.random.uniform(start_bounds[0][2], start_bounds[1][2]))
        g_point = (np.random.uniform(goal_bounds[0][0], goal_bounds[1][0]), 
                   np.random.uniform(goal_bounds[0][1], goal_bounds[1][1]),
                   np.random.uniform(goal_bounds[0][2], goal_bounds[1][2]))
        start_pose = Pose(Point(s_point), Quaternion(0,0,0,1))
        goal_pose = Pose(Point(g_point), Quaternion(0,0,0,1))

        # move to the start pose (beginning of the arc)
        print("planning to starting pose")
        
        plan, _ = demo.ur5.arm.compute_cartesian_path([start_pose], # waypoints to follow
										        0.01,       # eef_step  
										        0.0)        # jump_threshold  
        
        raw_input('Check Rviz for cartesian plan, press enter to execute')
        print("Start timing")
        start_time = time.time()
        
        demo.ur5.arm.execute(plan, wait=True)
        
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
        
        # demo.ur5.move_to_cartesian_goal(demo.grasp_goal.pose)

        # close gripper
        # raw_input("press enter to close gripper")
        # demo.gripper.send_gripper_command(commandName='close')
        # 138 smallest box feb10(fri)
        # 103 square-ish box feb10(fri)
        # 107 long box feb13(mon)


        demo.arc.original = demo.ur5.arm.get_current_pose()
        demo.arc.grasped = True
        
        waypoints, start_pose = demo.arc.plan_arc_trajectory_by_position(start_pose=start_pose, goal_pose=goal_pose)
        (plan, _) = demo.ur5.arm.compute_cartesian_path(waypoints, 0.01, 0.0)

        raw_input("check rviz before execute")
        # demo.ft_record = True
        demo.ur5.arm.execute(plan, wait=True)
        
        # demo.ft_record = False
        print('total work ', (demo.arc.translational_work + demo.arc.rotational_work))
        print('transalational work ', demo.arc.translational_work)
        print('rotational work ',  demo.arc.rotational_work)
        print('time', time.time()-start_time)

        demo.plot_ft()
        
        # open gripper
        # raw_input("press enter to open gripper")
        # demo.gripper.send_gripper_command(commandName=None, grip_width=0)
        
        # move up
        # end_goal = demo.ur5.arm.get_current_pose()
        # end_goal.pose.position.z += 0.03
        # demo.ur5.move_to_cartesian_goal(end_goal.pose)
        
        del demo
        
