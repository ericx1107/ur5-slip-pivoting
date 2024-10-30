#! /usr/bin/env python

import rospy
import message_filters
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import time
from slip_manipulation.msg import AngleStamped
from geometry_msgs.msg import WrenchStamped

class SyncData:
  def __init__(self):
    self.fz_array = []
    self.angle_array = []
    self.data = []
    self.time_array = []

    self.wrench_sub = message_filters.Subscriber('/robotiq_ft_wrench', WrenchStamped)
    self.angle_sub = message_filters.Subscriber('/slip_manipulation/rotation_angle', AngleStamped)

    self.ts = message_filters.ApproximateTimeSynchronizer([self.wrench_sub, self.angle_sub], 10, slop=0.0001)
    self.ts.registerCallback(self.callback)

  def callback(self, wrench_stamped, angle_stamped):
    fz = wrench_stamped.wrench.force.z
    angle = angle_stamped.angle
    tns = wrench_stamped.header.stamp.to_nsec()
    self.data.append([tns, angle, fz])
    self.fz_array.append(fz)
    self.angle_array.append(angle)
    self.time_array.append(tns)
    # print('callback')

if __name__ == "__main__":
    rospy.init_node("process_synced_data")
    
    proc = SyncData()

    # rospy.spin()
    while not rospy.is_shutdown():
      try:
        angle = rospy.wait_for_message('/slip_manipulation/rotation_angle', AngleStamped, timeout=rospy.Duration(20))
        # print('looping')
      except rospy.exceptions.ROSException:
        break

    fig, ax = plt.subplots()
    
    ax.scatter(proc.angle_array, proc.fz_array, s=(mpl.rcParams['lines.markersize'] ** 2)/8, c=proc.time_array, norm=mpl.colors.Normalize(), cmap='binary')
    ax.set_xlabel('Rotation (degrees)')
    ax.set_ylabel('Force in the z direction (N)')
    
    fig.savefig('/home/acrv/eric_ws/bagfiles/ft_' + str(time.time()) + '.png')

    with open("/home/acrv/eric_ws/bagfiles/data.csv", "wb") as f:
      writer = csv.writer(f)
      writer.writerows(proc.data)

    print("\nProcessed data.\n")