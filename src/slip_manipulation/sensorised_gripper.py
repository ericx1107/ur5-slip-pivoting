#! /usr/bin/env python

import rospy
import numpy as np
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from geometry_msgs.msg import WrenchStamped
from papillarray_ros_v2.msg import SensorState
from std_msgs.msg import Bool
import csv

class SensorisedGripper():
    def __init__(self):
        # set up gripper subscriber and publisher
        self.gripper_sub = rospy.Subscriber('/Robotiq2FGripperRobotInput', 
            inputMsg.Robotiq2FGripper_robot_input, self.gripper_callback)
        self.gripper_pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', 
            outputMsg.Robotiq2FGripper_robot_output, queue_size=1)

        # max gripper width 180
        self.grip_width = 0
        self.grip_max = 240
        self.grip_dist = 0.074 # m
        self.grip_inc = self.grip_max / self.grip_dist
        
        # set up tactile sensor subscribers
        self.tac0_data = SensorState()
        self.tac1_data = SensorState()

        self.tac0_data_arr = [['seq','gfX','gfY','gfZ','gtX','gtY','gtZ']]
        self.tac1_data_arr = [['seq','gfX','gfY','gfZ','gtX','gtY','gtZ']]

        self.tac0_sub = rospy.Subscriber('/hub_0/sensor_0', SensorState, self.sensor_callback, 0)
        self.tac1_sub = rospy.Subscriber('/hub_0/sensor_1', SensorState, self.sensor_callback, 1)
        
        self.tac0_contact = 0
        self.tac1_contact = 0

        # set up FT 300 sensor subscriber
        self.fts_data = WrenchStamped()
        self.fts_data_arr = [['Fx','Fy','Fz','Mx','My','Mz']]
        
        self.fts_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ftSensor_callback)

        # wait for subscribers to initialise and update data in class parameters
        rospy.sleep(1)

    def send_gripper_command(self, commandName="deactivate", grip_width=None):
        '''
        'commands' Starts with all zeros, but lets be careful in case of future changes
        shitty implementation of manual setting of grip width, feed None into commandName to use
        '''
        command = outputMsg.Robotiq2FGripper_robot_output()
        if (grip_width is not None) and (255 >= grip_width >= 0):
            command.rACT = 1
            command.rGTO = 1
            command.rPR = grip_width
            command.rSP  = 50
            command.rFR  = 150
        elif commandName=="deactivate":
            command.rACT = 0
            command.rGTO = 0
            command.rSP  = 50
            command.rGTO = 0
            command.rSP  = 50
            command.rFR  = 150
        elif commandName=="open":
            command.rACT = 1
            command.rGTO = 1
            command.rPR = 0
            command.rSP  = 50
            command.rFR  = 150
        elif commandName=="close":
            command.rACT = 1
            command.rGTO = 1
            command.rPR = 255
            command.rSP  = 50
            command.rFR  = 150
        
        # Command grippertac0_data
        self.gripper_pub.publish(command)

    def sensor_callback(self, data, sensor_num):
        # record all sensor data
        if sensor_num == 0:
            self.tac0_data = data
            # if self.tac0_data.is_contact:
            #     temp = [self.tac0_data.header.seq, self.tac0_data.gfX, self.tac0_data.gfY, self.tac0_data.gfZ,
            #     self.tac0_data.gtX, self.tac0_data.gtY, self.tac0_data.gtZ]
            #     self.tac0_data_arr.append(temp)

        else:
            self.tac1_data = data
            # if self.tac1_data.is_contact:
            #     temp = [self.tac1_data.header.seq, self.tac1_data.gfX, self.tac1_data.gfY, self.tac1_data.gfZ,
            #     self.tac1_data.gtX, self.tac1_data.gtY, self.tac1_data.gtZ]
            #     self.tac1_data_arr.append(temp)

    def gripper_callback(self, data):
        # record width of gripper
        self.grip_width = data.gPR

    def ftSensor_callback(self, data):
        self.fts_data = data
        temp = [self.fts_data.wrench.force.x, self.fts_data.wrench.force.y, self.fts_data.wrench.force.z, 
                self.fts_data.wrench.torque.x, self.fts_data.wrench.torque.y, self.fts_data.wrench.torque.z]
        self.fts_data_arr.append(temp)

    def manual_grip(self):
        print("current width: " + str(self.grip_width) + "\n")

        # manually enter grip width
        self.grip_width = int(raw_input("enter gripper width value\n"))

        if self.grip_width == -1:
            self.save_data()
            exit()

        self.send_gripper_command(None, self.grip_width)

        # check contact
        rospy.sleep(1)
        print("\n")
        if(self.tac0_data.is_contact and self.tac1_data.is_contact):
            print("both in contact")
        else:
            print("no contact")

        print("\n----------------------------------------")

        # wait for gripper to move
        rospy.sleep(1)

    def touch_object(self, box_dim, box_weight):
        # slowly tighten gripper until tactile sensors report contact
        
        both_contact = 0
        fg = 9.8 * box_weight
        print('fg', fg)
        long = rospy.wait_for_message('/slip_manipulation/is_long_edge', Bool, timeout=rospy.Duration(10))
        
        if long.data:
            # print("long is true")
            base_dim = box_dim[0]
            height_dim = box_dim[1]
        else:
            # print("long is false")
            base_dim = box_dim[1]
            height_dim = box_dim[0]
        init_ft_force = fg/2 * np.sin(np.pi/2 - np.arctan(height_dim/base_dim)) * np.cos(np.arctan(height_dim/base_dim))
        
        init_grip_width =  255 - int(self.grip_inc * box_dim[2])
        print('init grip width', init_grip_width)
        # raw_input("check grip width")
        # fully open gripper first
        # self.send_gripper_command(None, 0)
        # rospy.sleep(0.1)
        self.send_gripper_command(None, init_grip_width)
        rospy.sleep(2)


        while not both_contact:
            # increment grip tightness
            print('grip width', self.grip_width)
            if self.grip_width < init_grip_width + 15:
                self.grip_width = self.grip_width + 1
                self.send_gripper_command(None, self.grip_width)
                rospy.sleep(0.7)

            # update contact bool
            # for pillar in self.tac0_data.pillars:
            #     if pillar.in_contact:
            #         self.tac0_contact += 1
            # for pillar in self.tac1_data.pillars:
            #     if pillar.in_contact:
            #         self.tac1_contact += 1
            total_force = self.tac0_data.gfZ + self.tac1_data.gfZ
            # print('tac0_ contact', self.tac0_contact)
            # print('tac1_contact', self.tac1_contact)
            # print('force', 0.7 * total_force + init_ft_force)
            if self.tac0_data.is_contact and self.tac1_data.is_contact and 0.4 * total_force + init_ft_force > fg:
                both_contact = 1
            # else:
            #     self.tac0_contact, self.tac1_contact = 0, 0

        print("Grasped object.")
        return True

    def save_data(self):
        with open("./src/tactile_data/scripts/fts_data.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerows(self.fts_data_arr)

        with open("./src/tactile_data/scripts/tac0_data.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerows(self.tac0_data_arr)
        
        with open("./src/tactile_data/scripts/tac1_data.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerows(self.tac1_data_arr)

if __name__ == "__main__":
    pass
    