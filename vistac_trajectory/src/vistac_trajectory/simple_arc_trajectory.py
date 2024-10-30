#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
import copy
import rospy
import numpy as np
import tf2_ros
import time
import moveit_commander
import scipy
from geometry_msgs.msg import Pose, Point, Quaternion
from ur5_vistac_common.get_tf_helper import *
from std_msgs.msg import Bool,Float32
from geometry_msgs.msg import WrenchStamped
from papillarray_ros_v2.msg import SensorState
from control_msgs.msg import FollowJointTrajectoryActionGoal

class SimpleArc():
    # pose: Current Pose of the end effector
    # base_dim: box length that lays down on the ground
    # height_dim: box height value
    # long: boolean for if we are making the long direction of the box lay down on the ground
    # arm: "the move_group_commander", used for cartesian path
    # shift: the shifted value along the x direction
    
    '''
    Always pivots towards the centre of box
    '''
    def __init__(self, ur5_moveit, gripper):
        
        self.listener = tf.TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.ft_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft_callback)
        self.tac_sub = rospy.Subscriber('/hub_0/sensor_0', SensorState, self.tac_callback)
        self.gripper = gripper
        
        self.goal_pub = rospy.Publisher('/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)
        
        self.work_pub = rospy.Publisher('/slip_manipulation/work', Float32, queue_size=1)
        
        self.ur5 = ur5_moveit
        self.arm = ur5_moveit.arm
        self.robot = moveit_commander.RobotCommander()

        self.theta = 0
        self.tac_data = 0
        # self.Fz = 0
        # self.fg = 9.8 * box_weight * 1.5  # scaling shenanigans
        # self.box_angle = np.arctan(self.height_dim/self.base_dim)

        self.angle_err = 1e6 # set large number to initialise while loop
        self.threshold = 5e-1
        # PID constants
        self.Kp = 2e-4#1e-4
        self.Kd = 5e-4
        self.Ki = 8e-5#3e-6
        self.Kp_vision = 5e-1#1e-4
        self.Ki_vision = 1e-2#3e-6
        
        self.z_offset = 0
        self.x_offset = 0
        
        self.temp_loop = np.arange(5)
        
        self.dt = 0
        self.prev_err = 0
        self.intg_err = 0
        
        self.slip_type = 0
        self.slip_type_x = 0
        self.pillars_in_contact = 0
        self.safety_bound = 40
        
        self.prev_dt = time.time()
        
        self.original = None
        
        self.prev_angle = 0
        
        # self.start_z = 0
        # self.start_y = 0
        # self.start_theta = 0
        
        # self.end_z = 0
        # self.end_y = 0
        # self.end_theta = 0
        
        self.translational_work = 0
        self.rotational_work = 0
        self.grasped = False
        
        
        self.init_grip_width = None

    def plan_arc_trajectory_by_position(self, start_pose, goal_pose, num_waypoints=50):
        '''plan an arc trajectory along the circumference of a circle. The centre of the circle is 
        assumed to be the mid point between the start and the goal positions on the table. 
        '''
        
        waypoints = []
        
        # consider the centre of the arc to be on the table surface
        # find the radius as the distance to a point that is equidistant to the start and goal points,
        # constrained to be on the plane z=0 (on the table)
        s = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        g = np.array([goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])
        
        distance_func = lambda p: [
            np.sqrt((p[0] - s[0])**2 + (p[1] - s[1])**2 + s[2]**2) - 
            np.sqrt((p[0] - g[0])**2 + (p[1] - g[1])**2 + g[2]**2)
        ]

        initial_guess = [(s[0] + g[0]) / 2, (s[1] + g[1]) / 2]

        c = scipy.optimize.fsolve(distance_func, initial_guess)
        
        # calculate the radius of the arc as the distance between the centre and the start or goal point
        radius = np.sqrt((c[0] - s[0])**2 + (c[1] - s[1])**2 + s[2]**2)
        
        # calculate the start and the end angles for the given poses
        start_angle = np.arcsin(s[2] / radius)  # opposite / hypotenuse
        end_angle = np.arcsin(g[2] / radius)        

        # compute the intermediate waypoints between the start and end poses
        waypoint_pose = copy.deepcopy(start_pose)
        for theta in np.linspace(start_angle, end_angle, num_waypoints):
            waypoint_pose.position.z = s[2] + radius * np.sin(theta)
            waypoint_pose.position.y = s[1] + radius * np.cos(theta)
            
            # wpose_base = tf_transform_pose(self.listener, waypoint_pose, 'tool0', 'base_link').pose
            waypoints.append(copy.deepcopy(waypoint_pose))

        return waypoints, start_pose

    def plan_arc_trajectory_by_radius(self, radius, num_waypoints=50):
        '''plan an arc trajectory along the circumference of a circle with the given radius
        '''
        
        waypoints = []
        
        # consider the centre of the circle to be on the table surface next to the base origin (y=0, z=0)
        start_x = -0.40  # magic number on the table
        start_y = 0.0  # magic number on the table
        # use hard coded start and end angles
        start_angle = np.pi / 4
        end_angle = np.pi - start_angle
        # calculate the starting position given the radius of the curve
        # H = radius, theta = start_angle, O = z, A = y
        start_pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        start_pose.orientation = self.arm.get_current_pose().pose.orientation
        start_pose.position.z = radius * np.sin(start_angle)
        start_pose.position.y = start_y + radius * np.cos(start_angle)
        start_pose.position.x = start_x
        waypoint_pose = copy.deepcopy(start_pose)
        
        for theta in np.linspace(start_angle, end_angle, num_waypoints):
            waypoint_pose.position.z = radius * np.sin(theta)
            waypoint_pose.position.y = start_y + radius * np.cos(theta)
            
            # wpose_base = tf_transform_pose(self.listener, waypoint_pose, 'tool0', 'base_link').pose
            waypoints.append(copy.deepcopy(waypoint_pose))

        return waypoints, start_pose
        
        # old
        # Always assuming we start from the long direction
        # z is up and down, x is the direction of "home", y is the side  

        # waypoints = []
        # wpose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        
        # r = np.hypot(self.base_dim, self.height_dim)
        # box_angle = math.degrees(np.arctan(self.height_dim/self.base_dim))  + curr_angle
        
        #     # change is always positive
        #     # cos is always positive -> change * cos is positive -> x is positive
        #     # sin is always negative -> change * sin is negative -> y is negative
        #     # int(np.ceil(abs(goal_angle - curr_angle)))
        # for theta in np.linspace(box_angle, box_angle + goal_angle, 50):
        #     wpose.position.z = -(r*math.sin(math.radians(theta))) + self.height_dim # - 0.03
        #     wpose.position.y = self.base_dim - r*math.cos(math.radians(theta))
            
        #     wpose_base = tf_transform_pose(self.listener, wpose, 'tool0', 'base_link').pose
        #     waypoints.append(copy.deepcopy(wpose_base))
     
        # return waypoints
        
    def ft_callback(self, data):
        self.Fz = data.wrench.force.z
        self.Fy = data.wrench.force.y
        self.Tx = data.wrench.torque.x
        
        trans = patient_lookup_tf(self.tf_buffer, 'tool0')
        
        if self.grasped:
            # print("Fz is ", self.Fz)
            # print("Distance change is ", abs(trans.transform.translation.z - self.original.pose.position.z))
            self.translational_work  += abs(self.Fz * abs(trans.transform.translation.z - self.original.pose.position.z))
            self.translational_work  += abs(self.Fy * abs(trans.transform.translation.y - self.original.pose.position.y))
            
            self.original.pose.position.z = trans.transform.translation.z 
            self.original.pose.position.y = trans.transform.translation.y
        
        
            angle_diff = abs(self.theta - self.prev_angle) 
            self.prev_angle = self.theta
            
            self.rotational_work += abs((angle_diff) * self.Tx)
        
        
        
        
    def tac_callback(self, data):
        self.tac_data = data
        
        if self.init_grip_width is not None:
            self.slip_control(self.init_grip_width)
        
    # def force_pred(self):
    #     # force = 20/np.pi * np.arctan(-8 * (self.theta - np.pi/5)) - 6
    #     theta = np.pi/2 - self.box_angle - self.theta
    #     alpha = np.pi/2 - theta
    #     force = self.fg * np.sin(theta) * np.cos(alpha)/2
    #     return force


if __name__ == "__main__":
    pass
