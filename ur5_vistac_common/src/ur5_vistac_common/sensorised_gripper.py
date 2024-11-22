#! /usr/bin/env python

import rospy
import numpy as np
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from geometry_msgs.msg import WrenchStamped
from papillarray_ros_v2.msg import SensorState
from papillarray_ros_v2.srv import BiasRequest, BiasRequestRequest
from std_msgs.msg import Bool
import csv

class SensorisedGripper():
    '''Robotiq 2f85 gripper, with Robotiq FT300 force/torque sensor on the wrist,
    and Contactile Papillarrays on the fingertips
    '''
    def __init__(self, init_zero_tac=True):
        # set up gripper subscriber and publisher
        self.gripper_sub = rospy.Subscriber('/Robotiq2FGripperRobotInput', 
            inputMsg.Robotiq2FGripper_robot_input, self.gripper_callback)
        self.gripper_pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', 
            outputMsg.Robotiq2FGripper_robot_output, queue_size=1)

        # 2f85 takes gripper width inputs from 0-255 open to closed
        # max gripper width 180
        self.grip_width = 0
        self.grip_bound = 240   # gripper width that fully closes the grippers when tactile sensors are attached
        self.grip_dist = 0.074 # m
        self.grip_inc = self.grip_bound / self.grip_dist    # step/m
        
        # set up tactile sensor subscribers
        self.tac0_data = SensorState()
        self.tac1_data = SensorState()

        self.tac0_data_arr = [['seq','gfX','gfY','gfZ','gtX','gtY','gtZ']]
        self.tac1_data_arr = [['seq','gfX','gfY','gfZ','gtX','gtY','gtZ']]

        self.tac0_sub = rospy.Subscriber('/hub_0/sensor_0', SensorState, self.tac_sensor_callback, 0)
        self.tac1_sub = rospy.Subscriber('/hub_0/sensor_1', SensorState, self.tac_sensor_callback, 1)
        
        self.tac0_contact = 0
        self.tac1_contact = 0

        self.tac_bias_request = rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)

        # set up FT 300 sensor subscriber
        self.fts_data = WrenchStamped()
        self.fts_data_arr = [['Fx','Fy','Fz','Mx','My','Mz']]
        
        self.fts_sub = rospy.Subscriber('/robotiq_ft_wrench', WrenchStamped, self.ft_sensor_callback)

        # wait for subscribers to initialise and update data in class parameters
        rospy.sleep(1)
        
        self.safety_bound = 20
        self.max_grip_width = 255
        self.fric_coef = 0.15
        
        # zero tactile sensors
        if init_zero_tac:
            self.zero_tactile_sensors()

    def send_gripper_command(self, commandName="deactivate", grip_width=None):
        '''
        'commands' Starts with all zeros, but lets be careful in case of future changes
        shitty implementation of manual setting of grip width, feed None into commandName to use
        '''
        command = outputMsg.Robotiq2FGripper_robot_output()
        if (grip_width is not None) and (self.max_grip_width >= grip_width >= 0):
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

    def tac_sensor_callback(self, data, sensor_num):
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

    def ft_sensor_callback(self, data):
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

    # TODO: rewrite old touch_object function and move into a box-related script
    def touch_object(self, obj_width=None):
        '''slowly tighten gripper until tactile sensors report contact.
        input obj_width value (m) to give an init grip width to save time. Starts from open grip if None
        '''
        
        both_contact = False
        force_threshold = 5
        
        if obj_width is not None:
            init_grip_width =  self.grip_bound - int(self.grip_inc * obj_width) - self.safety_bound
        else:
            init_grip_width = 0
        
        print('init grip width: ' + str(init_grip_width))
        raw_input('Confirm grip width')
        self.send_gripper_command(None, init_grip_width)
        rospy.sleep(2)


        while not both_contact:
            # increment grip tightness
            print('grip width', self.grip_width)
            if self.grip_width < init_grip_width + self.safety_bound:
                self.grip_width = self.grip_width + 1
                self.send_gripper_command(None, self.grip_width)
                rospy.sleep(0.2)
            else: 
                rospy.logerr("Reached safety bound!")
                return None

            # update contact bool
            total_force = self.tac0_data.gfZ + self.tac1_data.gfZ
            print(total_force)
            
            # both exceed force threshold conditon
            if (self.tac0_data.gfZ > force_threshold) and (self.tac1_data.gfZ > force_threshold):
                both_contact = True
            
            # both contact condition
            # if self.tac0_data.is_contact and self.tac1_data.is_contact: # and self.fric_coef * total_force > init_ft_force:
                # both_contact = True

        print("Grasped object.")
        return init_grip_width

    def zero_tactile_sensors(self):
        request = BiasRequestRequest()
        threshold = 0.2
        attempt = 0
        state_names = ['gfX','gfY','gfZ','gtX','gtY','gtZ']
        state_values = np.array([getattr(self.tac0_data, n) for n in state_names] + [getattr(self.tac1_data, n) for n in state_names])
        
        # repeat a few times until the values are generally zero
        while not (state_values < threshold).all() and attempt < 10:
            # print(state_values)
            # print((state_values < threshold).all())
            try:
                response = self.tac_bias_request(request)
                rospy.loginfo("Bias result: " + str(response))
            except rospy.ServiceException as e: 
                rospy.logerr(str(e))
            attempt += 1
            rospy.sleep(0.5)

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
    