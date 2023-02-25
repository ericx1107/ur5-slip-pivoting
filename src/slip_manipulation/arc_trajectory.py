#!/usr/bin/env python
# use moveit_commander (the Python MoveIt user interfaces )
import copy
import rospy
import math
import numpy as np
import tf2_ros
import time
import moveit_commander
from geometry_msgs.msg import Pose, Point, Quaternion
from slip_manipulation.get_tf_helper import *
from std_msgs.msg import Bool,Float32
from geometry_msgs.msg import WrenchStamped
from slip_manipulation.msg import AngleStamped
from papillarray_ros_v2.msg import SensorState
from slip_manipulation.sensorised_gripper import SensorisedGripper
from control_msgs.msg import FollowJointTrajectoryActionGoal
from actionlib_msgs.msg import GoalStatusArray
from moveit_msgs.msg import RobotState

class ArcTrajectory():
    # pose: Current Pose of the end effector
    # base_dim: box length that lays down on the ground
    # height_dim: box height value
    # long: boolean for if we are making the long direction of the box lay down on the ground
    # arm: "the move_group_commander", used for cartesian path
    # shift: the shifted value along the x direction
    
    '''
    Always pivots towards the centre of box
    '''
    def __init__(self, box_dim, ur5_moveit, gripper, grasp_param, box_weight):
        self.long = rospy.wait_for_message('/slip_manipulation/is_long_edge', Bool, timeout=rospy.Duration(10))
        
        if self.long.data:
            # print("long is true")
            shift = (1 - abs(grasp_param)) * box_dim[0]/2
            self.base_dim = box_dim[0]-shift
            self.height_dim = box_dim[1]
            self.grasp_param = grasp_param
        else:
            # print("long is false")
            shift = (1 - abs(grasp_param)) * box_dim[1]/2
            self.base_dim = box_dim[1]-shift
            self.height_dim = box_dim[0]
            self.grasp_param = grasp_param
        
        # challenge offset
        self.base_dim += 0.05
        
        self.listener = tf.TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.angle_sub = rospy.Subscriber('/slip_manipulation/rotation_angle', AngleStamped, self.angle_callback)
        self.ft_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft_callback)
        # self.tac_sub = rospy.Subscriber('/hub_0/sensor_0', SensorState, self.tac_callback)
        # self.gripper_sub = rospy.Subscriber('/Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, self.gripper_callback)
        self.gripper = gripper
        # self.gripper_pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        
        self.goal_pub = rospy.Publisher('/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)
        
        self.work_pub = rospy.Publisher('/slip_manipulation/work', Float32, queue_size=1)
        
        
        
        self.ur5 = ur5_moveit
        self.arm = ur5_moveit.arm
        self.robot = moveit_commander.RobotCommander()

        self.theta = 0
        self.tac_data = 0
        # self.Fz = 0
        self.fg = 9.8 * box_weight * 1.5  # scaling shenanigans
        self.box_angle = np.arctan(self.height_dim/self.base_dim)

        self.angle_err = 1e6 # set large number to initialise while loop
        self.threshold = 5e-1
        # PID constants
        self.Kp = 2e-4#1e-4
        self.Kd = 5e-4
        self.Ki = 8e-5#3e-6
        self.Kp_vision = 2e-4#1e-4
        self.Ki_vision = 8e-5#3e-6
        
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

    def plan_cartesian_path(self, curr_angle, goal_angle):
        # Always assuming we start from the long direction
        # z is up and down, x is the direction of "home", y is the side  

        waypoints = []
        wpose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
        
        r = np.hypot(self.base_dim, self.height_dim)
        box_angle = math.degrees(np.arctan(self.height_dim/self.base_dim))  + curr_angle
        
            # change is always positive
            # cos is always positive -> change * cos is positive -> x is positive
            # sin is always negative -> change * sin is negative -> y is negative
            # int(np.ceil(abs(goal_angle - curr_angle)))
        for theta in np.linspace(box_angle, box_angle + goal_angle, 50):
            wpose.position.z = -(r*math.sin(math.radians(theta))) + self.height_dim # - 0.03
            wpose.position.y = self.base_dim - r*math.cos(math.radians(theta))
            
            wpose_base = tf_transform_pose(self.listener, wpose, 'tool0', 'base_link').pose
            waypoints.append(copy.deepcopy(wpose_base))
     
        return waypoints

    def angle_callback(self, data):
        self.theta = np.radians(data.angle)
        
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
        
    def force_pred(self):
        # force = 20/np.pi * np.arctan(-8 * (self.theta - np.pi/5)) - 6
        theta = np.pi/2 - self.box_angle - self.theta
        alpha = np.pi/2 - theta
        force = self.fg * np.sin(theta) * np.cos(alpha)/2
        return force

    def control_robot(self, goal_angle, init_grip_width=None):
        waypoints = self.plan_cartesian_path(self.theta, goal_angle)
        inc = 0     # also for faster goal publishing
        
        self.init_grip_width = init_grip_width
        
        while self.angle_err > 1 and waypoints != []:
            # self.start_theta = self.theta
            # self.start_z = self.arm.get_current_pose().pose.position.z
            # self.start_y = self.arm.get_current_pose().pose.position.y
            
            
            temp_waypoints = []
            
            
            print('z_offset: ', self.z_offset)
            # print('x_offset: ', self.x_offset)
            
            waypoints[0].position.z = waypoints[0].position.z + self.z_offset
            waypoints[0].position.x = waypoints[0].position.x + self.x_offset
            temp_waypoints.append(copy.deepcopy(waypoints[0]))
            waypoints.pop(0)
            # print(temp_waypoints)
            # rospy.sleep(0.3)
            self.dt = time.time()
            
            
            ################################### faster goal publishing ################################################
            # if inc == 0:
            #     # run normal plan for the first movement
            #     (plan, _) = self.arm.compute_cartesian_path(temp_waypoints, 0.01, 0.0)
                
            #     # set up goal message
            #     goal_msg = FollowJointTrajectoryActionGoal()
            #     goal_msg.goal_id.stamp = rospy.Time.now()
            #     goal_msg.goal.trajectory = plan.joint_trajectory
                
            #     # self.dt = time.time()
            # else:
            #     # use previous goal as start state
            #     start_state = RobotState()
            #     start_state.joint_state.header.frame_id = 'base_link'
            #     start_state.joint_state.header.stamp = rospy.Time.now()
            #     start_state.joint_state.name = plan.joint_trajectory.joint_names
            #     start_state.joint_state.position = plan.joint_trajectory.points[-1].positions
            #     start_state.joint_state.velocity = plan.joint_trajectory.points[-1].velocities
            #     start_state.joint_state.effort = plan.joint_trajectory.points[-1].effort
            #     start_state.is_diff = False
            #     (plan, _) = self.ur5.compute_cartesian_path_with_start_state(start_state, temp_waypoints, 0.01, 0.0)
                
            #     # set up goal
            #     goal_msg = FollowJointTrajectoryActionGoal()
            #     goal_msg.goal_id.stamp = rospy.Time.now()
            #     goal_msg.goal.trajectory = plan.joint_trajectory

            #     # self.dt = time.time()    
            #     # rospy.sleep(0.2)
            #     # wait for previous goal to complete
            #     # t = time.tsquare_short_open_without_2023-02-22-13-18-22 = rospy.wait_for_message('/scaled_pos_joint_traj_controller/follow_joint_trajectory/status', GoalStatusArray)
            #         except:
            #             pass
            #         # print('got status message')
            #         # print('goal_id',resp.status_list[-1].goal_id)
            #         # print('status',resp.status_list[-1].status)
                    
            #         if resp.status_list[-1].status == 3:
            #             # print('previous goal successful\n\n\n\n')
            #             # print(resp.status_list[-1].status)
            #             # print(time.time() - t)
            #             break
            
            # inc += 1

            
            # self.goal_pub.publish(goal_msg)
            ##########################################################################################################
            
            (plan, _) = self.arm.compute_cartesian_path(temp_waypoints, 0.01, 0.0)
            
            self.arm.execute(plan, wait=True)
            # execute_time = time.time() - execute_time
            # rospy.sleep(1.5)
            self.dt = time.time() - self.dt
            
            # get error between ideal and measured force
            predicted_force = self.force_pred()
            measured_force = self.Fz

            err = predicted_force - measured_force
            # print('error: ', err)
            # print("pred:", predicted_force)
            # print("meas:", measured_force)
            # print("error:", err)
            # print('dt: ', self.dt)
            # deriv_err = (err - self.prev_err) / self.dt 
            # accumulate integral error
            self.intg_err += err * self.dt
            # print("error:", err)
            # print("deriv error:", deriv_err)
            # print("intg error:", self.intg_err)
            
            # PI controller to minimise error
            # Error is positive when the robot is pushing too much, therefore the robot should raise its arm to reduce the force
            if err > self.threshold:
                # pass
                self.z_offset += ((self.Kp * err)) + (self.Ki * self.intg_err)
                # self.z_offset = ((self.Kp * err) + (self.Kd * deriv_err) + (self.Ki * self.intg_err)) * scale * abs(math.cos(2 * self.theta))
                # self.x_offset = ((self.Kp * err) + (self.Kd * deriv_err) + (self.Ki * self.intg_err)) * scale * math.sin(self.theta) * -np.sign(self.grasp_param)
            # Error is negative when the robot is lifting too much, therefore the robot should lower its arm to increase the force
            elif err < -self.threshold:
                # pass
                self.z_offset += (self.Kp * err) + (self.Ki * self.intg_err)
                # self.z_offset = ((self.Kp * err) + (self.Kd * deriv_err) + (self.Ki * self.intg_err)) * scale * abs(math.cos(2 * self.theta))
                # self.x_offset = ((self.Kp * err) + (self.Kd * deriv_err) + (self.Ki * self.intg_err)) * scale * math.sin(self.theta) * np.sign(self.grasp_param)
            # Do nothing
            else: 
                pass
            
            # print("Z offset: ", self.z_offset)
            # print("X offset: ", self.x_offset)

            
            self.angle_err = goal_angle - np.degrees(self.theta)
            self.prev_err = err
            
            
            # self.end_theta = self.theta
            # self.end_z = self.arm.get_current_pose().pose.position.z
            # self.end_y = self.arm.get_current_pose().pose.position.y
            
            
            
            # current_work_z = abs((self.end_z - self.start_z) * self.Fz)
            # current_work_y = abs((self.end_y - self.start_y) * self.Fy)
            
            
            # self.translational_work += current_work_z
            # self.translational_work += current_work_y
            # self.rotational_work += abs((self.start_theta - self.end_theta) * self.Tx)
            
            # print("Angle error: ", self.angle_err)
        
        
        # hack to stop gripper control
        self.init_grip_width =  None
        
        print("Pivot complete!")
        print("Transalational work", self.translational_work)
        print("Rotational work", self.rotational_work)
        
        self.work_pub.publish((self.rotational_work + self.translational_work))
        print("Total work", (self.rotational_work + self.translational_work))
        
    def slip_control(self, init_grip_width):
        stop_tightening = 1
        for pillar in self.tac_data.pillars:
            if abs(pillar.dX) > 8 or abs(pillar.dY) > 5 or abs(pillar.dZ) > 5:
                self.gripper.send_gripper_command(commandName = None, grip_width=self.gripper.grip_width - 1)
                stop_tightening = 0
                print("Loosening grip")
            if abs(pillar.dY) > 0.1 and np.sign(pillar.dY) == -1:
                self.slip_type += 1
            if abs(pillar.dX) > 0.1 and np.sign(pillar.dX) == -1:
                self.slip_type_x += 1
            if pillar.in_contact:
                self.pillars_in_contact += 1
        # print('slip type', self.slip_type)
        # print('pillars in contact', self.pillars_in_contact)
                
        if self.slip_type == self.pillars_in_contact or self.slip_type_x == self.pillars_in_contact: # Translational Slip
            if self.gripper.grip_width < init_grip_width + self.safety_bound and stop_tightening:
                self.gripper.send_gripper_command(commandName = None, grip_width=self.gripper.grip_width + 1)
                print('Tightening grip')
                if self.gripper.grip_width + 1 == init_grip_width + self.safety_bound:
                    print("Reached safety bound:", self.gripper.grip_width)        
                else:
                    print(self.gripper.grip_width)
        
        self.slip_type = 0
        self.pillars_in_contact = 0
        
    def vision_control_robot(self, goal_angle, init_grip_width=None):
        '''
        z offset from height from surface.
        do not run at the same time as control_robot
        '''
        
        # initialise ideal arc trajectory
        waypoints = self.plan_cartesian_path(self.theta, goal_angle)
        
        # set parameter for gripper control, no control if value is None
        self.init_grip_width = init_grip_width
        
        # loop through ideal trajectory waypoints, stop early if reached goal angle
        while goal_angle - np.degrees(self.theta) > 1 and waypoints != []:
            # PI controller to minimise box position error
            # get error between ideal and measured distance from surface
            lowest_vertex = rospy.wait_for_message('/slip_manipulation/lowest_vertex', Point)
            surface_height = 0.0    # hardcoded surface height
            height_err = lowest_vertex.z - surface_height
            
            # accumulate integral error, initially dt=0 so no error accumulated
            self.intg_err += height_err * self.dt
            
            # Error is positive when the robot is pushing too much, therefore the robot should raise its arm to reduce the force
            if height_err > self.threshold:
                # pass
                self.z_offset += ((self.Kp_vision * height_err)) + (self.Ki_vision * self.intg_err)
            # Error is negative when the robot is lifting too much, therefore the robot should lower its arm to increase the force
            elif height_err < -self.threshold:
                # pass
                self.z_offset += (self.Kp * height_err) + (self.Ki * self.intg_err)
            # Do nothing
            else: 
                pass
            
            # initialise time difference dt
            self.dt = time.time()
            
            print('z_offset: ', self.z_offset)
            
            waypoints[0].position.z = waypoints[0].position.z + self.z_offset
            waypoints[0].position.x = waypoints[0].position.x + self.x_offset


            # execute one waypoint
            (plan, _) = self.arm.compute_cartesian_path([waypoints[0]], 0.01, 0.0)
            waypoints.pop(0)
            
            self.arm.execute(plan, wait=True)

            # calculate dt for execution
            self.dt = time.time() - self.dt
        
        # hack to stop gripper control once pivot is finished
        self.init_grip_width = None
        
        # print work for recording results
        print("Pivot complete!")
        print("Transalational work", self.translational_work)
        print("Rotational work", self.rotational_work)
        
        self.work_pub.publish((self.rotational_work + self.translational_work))
        print("Total work", (self.rotational_work + self.translational_work))            

if __name__ == "__main__":
    pass
