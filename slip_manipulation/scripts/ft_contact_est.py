#! /usr/bin/env python

import rospy
import numpy as np
from robotiq_ft_sensor.msg import ft_sensor
import time

class ContactEstimator():
    def __init__(self,obj_wt):
        rospy.init_node('ft_contact_est')
        
        self.fts = ft_sensor()
        self.fts_sub = rospy.Subscriber('/robotiq_ft_sensor', ft_sensor, self.ftSensor_callback)
        
        self.initial_flag = 1
        self.init_force = 0
        self.obj_f = obj_wt * 9.8
        self.count = 0
        rospy.sleep(1)
        
    def ftSensor_callback(self, data):
        self.fts = data
        
    def initial_force(self):
        if self.initial_flag:
            rospy.sleep(1)
            fX_temp, fY_temp, fZ_temp = [],[],[]
            for i in range(20):
                fZ_temp.append(self.fts.Fz)
            self.init_force = np.average(fZ_temp)
            self.initial_flag = 0
        return self.init_force
    
    def contact_state(self):
        
        self.count += 1
        print("initial force is", self.init_force)
        print("force in x is ", self.fts.Fx)
        print("force in y is ", self.fts.Fy)
        print("force in z is ", self.fts.Fz)
        print("first condition is ", self.init_force + self.obj_f)

        
        
        
        if (self.init_force + self.obj_f) * 0.95 <= self.fts.Fz:
            print('No contact')
        else:
            print('Contact')
            
if __name__ == "__main__":
    
    ce = ContactEstimator(1.276)
    ce.initial_force()
    while not rospy.is_shutdown():
        ce.contact_state()
        rospy.sleep(3)
    