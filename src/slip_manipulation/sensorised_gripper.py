#! /usr/bin/env python

import rospy

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_ft_sensor.msg import ft_sensor
from papillarray_ros_v2.msg import SensorState
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

        # set up tactile sensor subscribers
        self.tac0_data = SensorState()
        self.tac1_data = SensorState()

        self.tac0_data_arr = [['seq','gfX','gfY','gfZ','gtX','gtY','gtZ']]
        self.tac1_data_arr = [['seq','gfX','gfY','gfZ','gtX','gtY','gtZ']]

        self.tac0_sub = rospy.Subscriber('/hub_0/sensor_0', SensorState, self.sensor_callback, 0)
        self.tac1_sub = rospy.Subscriber('/hub_0/sensor_1', SensorState, self.sensor_callback, 1)

        # set up FT 300 sensor subscriber
        self.fts_data = ft_sensor()
        self.fts_data_arr = [['Fx','Fy','Fz','Mx','My','Mz']]
        
        self.fts_sub = rospy.Subscriber('/robotiq_ft_sensor', ft_sensor, self.ftSensor_callback)

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
            if self.tac0_data.is_contact:
                temp = [self.tac0_data.header.seq, self.tac0_data.gfX, self.tac0_data.gfY, self.tac0_data.gfZ,
                self.tac0_data.gtX, self.tac0_data.gtY, self.tac0_data.gtZ]
                self.tac0_data_arr.append(temp)

        else:
            self.tac1_data = data
            if self.tac1_data.is_contact:
                temp = [self.tac1_data.header.seq, self.tac1_data.gfX, self.tac1_data.gfY, self.tac1_data.gfZ,
                self.tac1_data.gtX, self.tac1_data.gtY, self.tac1_data.gtZ]
                self.tac1_data_arr.append(temp)

    def gripper_callback(self, data):
        # record width of gripper
        self.grip_width = data.gPR

    def ftSensor_callback(self, data):
        self.fts_data = data
        temp = [self.fts_data.Fx, self.fts_data.Fy, self.fts_data.Fz, self.fts_data.Mx, self.fts_data.My,
        self.fts_data.Mz]
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

    def touch_object(self):
        # slowly tighten gripper until tactile sensors report contact

        both_contact = self.tac0_data.is_contact and self.tac1_data.is_contact

        # fully open gripper first
        self.send_gripper_command(None, 0)

        while not both_contact:
            # increment grip tightness
            if self.grip_width < 100:
                self.grip_width = self.grip_width + 1
                self.send_gripper_command(None, self.grip_width)
                rospy.sleep(0.1)

            # update contact bool
            both_contact = self.tac0_data.is_contact and self.tac1_data.is_contact

        print("gentle touch on object.")
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
    