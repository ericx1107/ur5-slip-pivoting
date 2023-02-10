#! /usr/bin/env python

import rospy
import message_filters
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.lines import Line2D
from slip_manipulation.msg import AngleStamped
from geometry_msgs.msg import WrenchStamped
from papillarray_ros_v2.msg import SensorState
import time

class SyncData:
  def __init__(self):
    self.fx_array, self.fy_array, self.fz_array = [], [], []
    self.angle_array = []
    self.data = []
    self.time_array = []

    self.wrench_sub = message_filters.Subscriber('/robotiq_ft_wrench', WrenchStamped)
    self.angle_sub = message_filters.Subscriber('/slip_manipulation/rotation_angle', AngleStamped)
    self.tactile_sub = message_filters.Subscriber('/hub_0/sensor_0', SensorState)

    self.ts = message_filters.ApproximateTimeSynchronizer([self.tactile_sub, self.angle_sub], 10, slop=0.0001)
    # self.ts = message_filters.ApproximateTimeSynchronizer([self.tactile_sub, self.angle_sub], 10, slop=0.0001)
    self.ts.registerCallback(self.callback)

  def callback(self, sensor_state, angle_stamped):
    fx = sensor_state.gfX
    fy = sensor_state.gfY
    fz = sensor_state.gfZ
    # fz = sensor_state.wrench.force.z
    angle = angle_stamped.angle
    tns = sensor_state.header.stamp.to_nsec()
    self.data.append([tns, angle, fx, fy, fz])
    # self.data.append([tns, angle, fz])
    self.fx_array.append(fx)
    self.fy_array.append(fy)
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

    # fig, ax = plt.subplots()
    
    # ax.scatter(proc.angle_array, proc.fx_array, s=(mpl.rcParams['lines.markersize'] ** 2)/2, c=proc.time_array, norm=mpl.colors.Normalize(), cmap='binary')
    # ax.set_xlabel('Rotation (degrees)')
    # ax.set_ylabel('Force in the x direction (N)')
    
    # fig.savefig('/home/acrv/eric_ws/bagfiles/data_fX.png')
    
    # fig, ax = plt.subplots()
    
    # ax.scatter(proc.angle_array, proc.fy_array, s=(mpl.rcParams['lines.markersize'] ** 2)/2, c=proc.time_array, norm=mpl.colors.Normalize(), cmap='binary')
    # ax.set_xlabel('Rotation (degrees)')
    # ax.set_ylabel('Force in the y direction (N)')
    
    # fig.savefig('/home/acrv/eric_ws/bagfiles/data_fY.png')
    
    # fig, ax = plt.subplots()
    
    # ax.scatter(proc.angle_array, proc.fz_array, s=(mpl.rcParams['lines.markersize'] ** 2)/2, c=proc.time_array, norm=mpl.colors.Normalize(), cmap='binary')
    # ax.set_xlabel('Rotation (degrees)')
    # ax.set_ylabel('Force in the z direction (N)')
    
    # fig.savefig('/home/acrv/eric_ws/bagfiles/data_fZ.png')
    
    fig, ax = plt.subplots()
    
    ax.scatter(proc.angle_array, proc.fx_array, s=(mpl.rcParams['lines.markersize'] ** 2)/2, c=proc.time_array, norm=mpl.colors.Normalize(), cmap='Reds', label='Fx')
    ax.scatter(proc.angle_array, proc.fy_array, s=(mpl.rcParams['lines.markersize'] ** 2)/2, c=proc.time_array, norm=mpl.colors.Normalize(), cmap='Blues', label='Fy')
    ax.scatter(proc.angle_array, proc.fz_array, s=(mpl.rcParams['lines.markersize'] ** 2)/2, c=proc.time_array, norm=mpl.colors.Normalize(), cmap='Greens', label='Fz')
    ax.set_xlabel('Rotation (degrees)')
    ax.set_ylabel('Force in the z direction (N)')
    legend_elements = [Line2D([0], [0], marker = 'o', color = 'r', markerfacecolor = 'r', label = 'Fx'), 
                       Line2D([0], [0], marker = 'o', color = 'b', markerfacecolor = 'b', label = 'Fy'),
                       Line2D([0], [0], marker = 'o', color = 'y', markerfacecolor = 'g', label = 'Fz')]
    ax.legend(handles = legend_elements, loc='best')
    fig.savefig('/home/acrv/eric_ws/bagfiles/tactile_box_3_long_pivot_' + str(time.time()) + '.png')

    with open("/home/acrv/eric_ws/bagfiles/data.csv", "wb") as f:
      writer = csv.writer(f)
      writer.writerows(proc.data)

    print("\nProcessed data.\n")