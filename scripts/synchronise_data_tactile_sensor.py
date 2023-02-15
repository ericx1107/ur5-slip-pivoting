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
    self.dx_array, self.dy_array, self.dz_array = [[] for x in range(9)], [[] for x in range(9)], [[] for x in range(9)]
    self.angle_array = [[] for x in range(9)]
    self.data = []
    self.time_array = [[] for x in range(9)]

    self.wrench_sub = message_filters.Subscriber('/robotiq_ft_wrench', WrenchStamped)
    self.angle_sub = message_filters.Subscriber('/slip_manipulation/rotation_angle', AngleStamped)
    self.tactile_sub = message_filters.Subscriber('/hub_0/sensor_0', SensorState)

    self.ts = message_filters.ApproximateTimeSynchronizer([self.tactile_sub, self.angle_sub], 10, slop=0.0001)
    # self.ts = message_filters.ApproximateTimeSynchronizer([self.tactile_sub, self.angle_sub], 10, slop=0.0001)
    self.ts.registerCallback(self.callback)

  def callback(self, sensor_state, angle_stamped):
    for idx, pillar in enumerate(sensor_state.pillars):
      dx = pillar.dX
      dy = pillar.dY
      dz = pillar.dZ
      # fz = sensor_state.wrench.force.z
      angle = angle_stamped.angle
      tns = sensor_state.header.stamp.to_nsec()
      self.data.append([tns, angle, dx, dy, dz])
      # self.data.append([tns, angle, fz])
      self.dx_array[idx - 1].append(dx)
      self.dy_array[idx - 1].append(dy)
      self.dz_array[idx - 1].append(dz)
      self.angle_array[idx - 1].append(angle)
      self.time_array[idx - 1].append(tns)
      
    # print('callback')

if __name__ == "__main__":
    rospy.init_node("process_synced_data")
    
    proc = SyncData()

    # rospy.spin()
    while not rospy.is_shutdown():
      try:
        angle = rospy.wait_for_message('/slip_manipulation/rotation_angle', AngleStamped, timeout=rospy.Duration(10))
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
    fig, axs = plt.subplots(3, 3)
    idx = 1
    for id1 in range(3):
      for id2 in range(3):
        # print(proc.angle_array[idx - 1])
        axs[id1, id2].scatter(proc.angle_array[idx - 1], proc.dx_array[idx - 1], s=(mpl.rcParams['lines.markersize'] ** 2)/4, c=proc.time_array[0], norm=mpl.colors.Normalize(), cmap='Reds', label='dx')
        axs[id1, id2].scatter(proc.angle_array[idx - 1], proc.dy_array[idx - 1], s=(mpl.rcParams['lines.markersize'] ** 2)/4, c=proc.time_array[0], norm=mpl.colors.Normalize(), cmap='Blues', label='dy')
        axs[id1, id2].scatter(proc.angle_array[idx - 1], proc.dz_array[idx - 1], s=(mpl.rcParams['lines.markersize'] ** 2)/4, c=proc.time_array[0], norm=mpl.colors.Normalize(), cmap='Greens', label='dz')
        axs[2, id2].set_xlabel('Rotation (degrees)')
        axs[id1, 0].set_ylabel('Displacement (mm)')
        legend_elements = [Line2D([0], [0], marker = 'o', color = 'r', markerfacecolor = 'r', label = 'dx'), 
                          Line2D([0], [0], marker = 'o', color = 'b', markerfacecolor = 'b', label = 'dy'),
                          Line2D([0], [0], marker = 'o', color = 'y', markerfacecolor = 'g', label = 'dz')]
        axs[id1, id2].legend(handles = legend_elements, loc='best')
        idx += 1
    fig.set_size_inches(18.5, 10.5)
    fig.savefig('/home/acrv/eric_ws/bagfiles/tactile_plots/tactile_box_3_long_pivot_translational_' + str(time.time()) + '.png', dpi = 100)
    
    

    print("\nProcessed data.\n")