#! /usr/bin/env python

import rospy
import message_filters
import csv
import numpy as np
import matplotlib.pyplot as plt
from slip_manipulation.msg import AngleStamped
from geometry_msgs.msg import WrenchStamped

class SyncData:
  def __init__(self):
    self.fz_array = []
    self.angle_array = []
    self.data = []
    self.inc = 0

    self.wrench_sub = message_filters.Subscriber('/robotiq_ft_wrench', WrenchStamped)
    self.angle_sub = message_filters.Subscriber('/slip_manipulation/rotation_angle', AngleStamped)

    self.ts = message_filters.ApproximateTimeSynchronizer([self.wrench_sub, self.angle_sub], 10, slop=0.00005)
    self.ts.registerCallback(self.callback)

  def callback(self, wrench_stamped, angle_stamped):
    self.inc = 0  # reset increment

    fz = wrench_stamped.wrench.force.z
    angle = angle_stamped.angle
    self.data.append([fz, angle])
    self.fz_array.append(fz)
    self.angle_array.append(angle)
    print('callback')

if __name__ == "__main__":
    rospy.init_node("process_synced_data")
    
    proc = SyncData()

    # rospy.spin()
    while not rospy.is_shutdown():
      try:
        angle = rospy.wait_for_message('/slip_manipulation/rotation_angle', AngleStamped, timeout=rospy.Duration(5))
        # print('looping')
      except rospy.exceptions.ROSException:
        break

    fig, ax = plt.subplots()
    
    ax.scatter(proc.angle_array, proc.fz_array)
    ax.set_xlabel('Rotation (degrees)')
    ax.set_ylabel('Force in the z direction (N)')
    
    fig.savefig('/home/acrv/eric_ws/bagfiles/data.png')

    with open("/home/acrv/eric_ws/bagfiles/data.csv", "wb") as f:
      writer = csv.writer(f)
      writer.writerows(proc.data)

    