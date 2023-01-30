#! /usr/bin/env python

import rospy
from robotiq_ft_sensor.msg import ft_sensor
from slip_manipulation.msg import FtSensorStamped

def callback(data, publisher):
    stamped = FtSensorStamped()
    stamped.header.stamp = rospy.Time.now()
    stamped.header.frame_id = 'base_link'
    stamped.ft_data = data
    
    publisher.publish(stamped)

if __name__ == "__main__":
    rospy.init_node("ft_sensor_republisher")
    
    pub = rospy.Publisher('/slip_manipulation/stamped_ft_data', FtSensorStamped, queue_size=1)    
    sub = rospy.Subscriber('/robotiq_ft_sensor', ft_sensor, callback, callback_args=pub)
    
    rospy.spin()