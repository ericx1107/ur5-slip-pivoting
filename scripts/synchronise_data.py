#! /usr/bin/env python

import rospy
import message_filters
import csv
import numpy as np
from slip_manipulation.msg import AngleStamped
from geometry_msgs.msg import WrenchStamped

class SyncData:
  def __init__(self):
    self.fz_array = []
    self.angle_array = []
    self.inc = 0

    wrench_sub = message_filters.Subscriber('robotiq_ft_wrench', WrenchStamped)
    angle_sub = message_filters.Subscriber('/slip_manipulation/rotation_angle', AngleStamped)

    ts = message_filters.TimeSynchronizer([wrench_sub, angle_sub], 10)
    ts.registerCallback(self.callback)

  def callback(self, wrench_stamped, angle_stamped):
    self.inc = 0  # reset increment

    fz = wrench_stamped.wrench.force.z
    angle = angle_stamped.angle

    self.fz_array.append(fz)
    self.angle_array.append(angle)

if __name__ == "__main__":
    rospy.init_node("process_synced_data")
    
    proc = SyncData()

    while not rospy.is_shutdown():
      angle = rospy.wait_for_message('/slip_manipulation/rotation_angle', AngleStamped, timeout=rospy.Duration(5))

      if angle is None:
        break

    with open("/home/acrv/eric_ws/bagfiles/data.csv", "wb") as f:
      writer = csv.writer(f)
      writer.writerows(np.concatenate(proc.angle_array, proc.fz_array))

    