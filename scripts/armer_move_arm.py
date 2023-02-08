#! /usr/bin/env python

import rospy
import time
import numpy as np
import math
import copy
from geometry_msgs.msg import WrenchStamped
from armer_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from std_msgs.msg import Empty, Float32
from slip_manipulation.arc_trajectory import ArcTrajectory
from slip_manipulation.ur5_armer import UR5Armer

def callback(data, args):
    print("Starting arc trajectory")
    armer = args[0]
    traj = args[1]
    end_pub = args[2]
    
    # do some shit with control or whatever
    control_robot_armer(armer, traj, 90)
    # finish and publish end indicator message
    end_pub.publish()
    
def control_robot_armer(armer, arc, goal_angle):
    waypoints = arc.plan_cartesian_path(arc.theta, goal_angle)
    waypoints.pop(0)
    arc.time = time.time()
    while arc.angle_err > 2:
        # execute waypoints one at a time
        ##############################################################################
        # apply offset to waypoint
        waypoints[0].position.z = waypoints[0].position.z + arc.z_offset
        waypoints[0].position.x = waypoints[0].position.x + arc.x_offset
        print(waypoints[0])
        # move to waypoint
        armer.armer_move_to_pose_goal(waypoints[0])
        
        # remove used waypoint
        waypoints.pop(0)
        ################################################################################
        # how tf does this work
        
        # temp_waypoints = []
        # loop_length = np.arange(len(waypoints)) if len(waypoints) < 3 else np.arange(3)
        # for i in loop_length:
        #     waypoints[0].position.z = waypoints[0].position.z + arc.z_offset
        #     waypoints[0].position.x = waypoints[0].position.x + arc.x_offset
        #     temp_waypoints.append(copy.deepcopy(waypoints[0]))
        #     waypoints.pop(0)
        
        # (plan, _) = arc.arm.compute_cartesian_path(temp_waypoints, 0.01, 0.0)
        # arc.arm.execute(plan, wait=True)
        
        predicted_force = arc.force_pred()
        
        temp_force = []
        for i in arc.temp_loop:
            wrench_stamped = rospy.wait_for_message('/robotiq_ft_wrench', WrenchStamped, timeout=rospy.Duration(5)).wrench.force.z
            temp_force.append(wrench_stamped)
        measured_force = np.average(temp_force)
        err = predicted_force - measured_force
        # print("pred:", predicted_force)
        # print("meas:", measured_force)
        # print("error:", err)
        arc.dt = time.time() - arc.time
        deriv_err = (err - arc.prev_err) / arc.dt 
        
        # Error is positive when the robot is pushing too much, therefore the robot should raise its arm to reduce the force
        if err > arc.threshold:
            arc.z_offset = ((arc.Kp * err) + (arc.Kd * deriv_err)) * math.sin(arc.theta)
            arc.x_offset = ((arc.Kp * err) + (arc.Kd * deriv_err)) * math.cos(arc.theta) * np.sign(arc.grasp_param)
        # Error is negative when the robot is lifting too much, therefore the robot should lower its arm to increase the force
        elif err < -arc.threshold:
            arc.z_offset = ((arc.Kp * err) + (arc.Kd * deriv_err)) * math.sin(arc.theta)
            arc.x_offset = ((arc.Kp * err) + (arc.Kd * deriv_err)) * math.cos(arc.theta) * -np.sign(arc.grasp_param)
        # Do nothing
        else: 
            arc.z_offset = 0
            arc.x_offset = 0
        # print("Z offset: ", arc.z_offset)
        # print("X offset: ", arc.x_offset)
        
        # plan = arc.move_control_robot(arc.z_offset, arc.x_offset)
        
        arc.angle_err = goal_angle - np.degrees(arc.theta)
        arc.time = time.time()
        arc.prev_err = err

        print("Pivot complete!")


if __name__ == "__main__":
    rospy.init_node('armer_execute_trajectory')
    armer = UR5Armer()
    
    box_dim = [0.18, 0.11, 0.04]
    box_weight = 1.276

    grasp_param = rospy.wait_for_message('slip_manipulation/grasp_param', Float32).data
    print("got grasp param =", grasp_param)
    
    traj = ArcTrajectory(box_dim, grasp_param, box_weight)
    end_pub = rospy.Publisher('slip_manipulation/start_end_sequence', Empty, queue_size=1)
    rospy.Subscriber('slip_manipulation/start_armer_manipulation', Empty, callback, callback_args=(armer, traj, end_pub))
    
    print("waiting for signal")
    rospy.spin()