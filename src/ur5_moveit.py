#! /usr/bin/env python

import rospy
import moveit_commander
import numpy as np
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs

class UR5Moveit():
    
    def __init__(self):

        # initialise moveit planning scene
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.table_size = [2, 2, 0.87]

        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        # self.arm.set_planner_id("") # /home/acrv/HRIGroupAdmin/example_ros_ws/src/universal_robot/ur5_moveit_config/config/ompl_planning.yaml

        self.init_planning_scene()

        # initialise joint constraints to prevent collision with camera mount
        self.init_moveit_constraints()

        # define important poses
        # self.arm.set_named_target("up") # go to up position if not already there
        self.start_pose = {
            'shoulder_pan_joint': 0,
            'shoulder_lift_joint': -np.pi/2,
            'elbow_joint': -np.pi/2,
            'wrist_1_joint': -np.pi/2,
            'wrist_2_joint': np.pi/2,
            'wrist_3_joint': np.pi
        }

        # move to start pose
        # self.move_to_joints_pose(self.start_pose)

    def init_planning_scene(self):
        # add table collision object
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = box_pose.pose.position.z - .44 # shift by table size/2
        box_name = "table"
        self.scene.add_box(box_name, box_pose, size=self.table_size)

    def init_moveit_constraints(self):
        self.camera_constraints = Constraints()
        self.camera_constraints.name = 'camera'
        
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'wrist_2_joint'
        joint_constraint.position = 0
        joint_constraint.tolerance_above = 2.55
        joint_constraint.tolerance_below = 2.55
        joint_constraint.weight = 1
        
        self.camera_constraints.joint_constraints.append(joint_constraint)

        self.arm.set_path_constraints(self.camera_constraints)
        

    def move_to_joints_pose(self, goal_pose):
        self.arm.set_joint_value_target(goal_pose)

        plan = self.arm.plan()

        raw_input('Check Rviz for plan, press enter to execute')

        self.arm.execute(plan)

    def tf_transform_pose(self, input_pose, from_frame, to_frame):
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # make PoseStamped message from Pose input
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1), PoseStamped)
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

if __name__ == "__main__":
    pass
