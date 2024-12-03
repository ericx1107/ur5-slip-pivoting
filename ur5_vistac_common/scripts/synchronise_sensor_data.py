#! /usr/bin/env python

import rospy
import message_filters
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import os
import cv2
from datetime import datetime, tzinfo, timedelta
from pathlib import Path
from matplotlib.lines import Line2D
from slip_manipulation.msg import AngleStamped
from geometry_msgs.msg import WrenchStamped, PoseStamped
from papillarray_ros_v2.msg import SensorState
from sensor_msgs import Image, CompressedImage
import time

class SyncData():
  '''synchronises the different data sources to a fixed uniform rate.
  handles contactile tactile sensors (both fingertip arrays), wrist f/t sensors and camera streams.
  the synchronised data are stored in class variables.

  input parameters
  sync_slop: the delay (in seconds) with which messages can be synchronised
  '''
  def __init__(self, save_dir, sync_slop=0.0001):
    # inits
    # self.ft_fx_array, self.ft_ft_array, self.ft_fz_array = [], [], []
    self.ft_f_array = []
    self.ft_t_array = []
    # self.tac0_dx_array, self.tac0_dy_array, self.tac0_dz_array = [], [], []
    # self.tac0_fx_array, self.tac0_fy_array, self.tac0_fz_array = [], [], []
    # self.tac1_dx_array, self.tac1_dy_array, self.tac1_dz_array = [], [], []
    # self.tac1_fx_array, self.tac1_fy_array, self.tac1_fz_array = [], [], []
    self.tac0_d_array = []
    self.tac0_f_array = []
    self.tac1_d_array = []
    self.tac1_f_array = []

    self.hand_cam_rgb_array = []
    self.hand_cam_depth_array = []
    self.side_cam_rgb_array = []

    self.time_array = []

    # init save paths
    # make a folder for the experiment
    class AEST(tzinfo):
        def utcoffset(self, dt):
            return timedelta(hours=10)
        def tzname(self, dt):
            return "AEST"
        def dst(self, dt):
            return timedelta(hours=11)
    aest = AEST()
    t = datetime.now(aest)
    dt_string = t.strftime("%Y_%m_%d___%H_%M_%S")
    
    self.exp_dir = save_dir / Path(dt_string)
    os.mkdir(self.exp_dir)   # do not allow same name directories
    # make a folder for the individual modalities
    self.dir_names = ['wrist_ft_force', 'wrist_ft_torque', 
            'tac0_displacement', 'tac1_displacement', 'tac0_force', 'tac1_force', 
            'in_hand_rgb', 'in_hand_depth', 'side_rgb']
    for dir in self.dir_names:
      os.makedirs(self.exp_dir / dir)

    # subscribers
    self.wrench_sub = message_filters.Subscriber('/robotiq_ft_wrench', WrenchStamped) # 60hz
    self.tac0_sub = message_filters.Subscriber('/hub_0/sensor_0', SensorState)  # 500hz
    self.tac1_sub = message_filters.Subscriber('/hub_0/sensor_1', SensorState)
    self.hand_cam_rgb_sub = message_filters.Subscriber('/d405/color/image_raw', Image)  # 60hz
    # self.hand_cam_rgb_sub = message_filters.Subscriber('/d405/color/image_raw/compressed', CompressedImage)
    self.hand_cam_depth_sub = message_filters.Subscriber('/d405/depth/image_rect_raw', Image) # 60hz
    # self.hand_cam_depth_sub = message_filters.Subscriber('/d405/depth/image_rect_raw/compressed', CompressedImage)
    self.side_cam_rgb_sub = message_filters.Subscriber('/d435/color/image_raw', Image)  # 60hz
    self.mocap_obj_sub = message_filters.Subscriber('/object/pose', PoseStamped)

    self.ts = message_filters.ApproximateTimeSynchronizer(
      [self.wrench_sub,
       self.tac0_sub, self.tac1_sub, 
       self.hand_cam_rgb_sub, self.hand_cam_depth_sub, self.side_cam_rgb_sub], 
      queue_size=10, slop=sync_slop)
    self.ts.registerCallback(self.callback)

  def callback(self, ft_wrench, tac0, tac1, hand_rgb, hand_depth, side_rgb):
    # process wrist ft sensor forces
    self.ft_f_array.append([ft_wrench.wrench.force.x, ft_wrench.wrench.force.y, ft_wrench.wrench.force.z])
    self.ft_t_array.append([ft_wrench.wrench.torque.x, ft_wrench.wrench.torque.y, ft_wrench.wrench.torque.z])
    
    # process tactile sensors forces
    pillars_d = []
    pillars_f = []
    for pillar in tac0.pillars:
      # loop through individual pillars
      # collate values into arrays 3(xyz) by 9(pillars)
      pillars_d.append([pillar.dX, pillar.dY, pillar.dZ])
      pillars_f.append([pillar.fX, pillar.fY, pillar.fZ])
    # add pillars data into main array
    self.tac0_d_array.append(pillars_d)
    self.tac0_f_array.append(pillars_f)
    
    pillars_d = []
    pillars_f = []
    for pillar in tac1.pillars:
      # loop through individual pillars
      # collate values into arrays 3(xyz) by 9(pillars)
      pillars_d.append([pillar.dX, pillar.dY, pillar.dZ])
      pillars_f.append([pillar.fX, pillar.fY, pillar.fZ])
    # add pillars data into main array
    self.tac1_d_array.append(pillars_d)
    self.tac1_f_array.append(pillars_f)
    # resulting np array shape will be Nx9x3
    
    # process in hand camera images
    # save into big numpy array/list and loop through later? might be too large to do live
    self.hand_cam_rgb_array.append(hand_rgb)
    self.hand_cam_depth_array.append(hand_depth)
    
    # process side camera images
    self.side_cam_rgb_array.append(side_rgb)

    # process time stamps?
    tns = tac0.header.stamp.to_nsec()
    self.time_array.append(tns)
    if self.time_array:
      print(float(tns) - float(self.time_array[-1]))

  def save_data(self):
    # loop through each frame
    for i in range(len(self.ft_f_array)):
      # save wrist ft
      np.save(self.exp_dir / self.dir_names[0] / str(i), self.ft_f_array[i])
      np.save(self.exp_dir / self.dir_names[1] / str(i), self.ft_t_array[i])
    
      # save tactile sensors
      np.save(self.exp_dir / self.dir_names[2] / str(i), self.tac0_d_array[i])
      np.save(self.exp_dir / self.dir_names[3] / str(i), self.tac1_d_array[i])
      np.save(self.exp_dir / self.dir_names[4] / str(i), self.tac0_f_array[i])
      np.save(self.exp_dir / self.dir_names[5] / str(i), self.tac1_f_array[i])
    
      # save camera images
      # save the image as rgb or binary
      cv2.imwrite(str(self.exp_dir / self.dir_names[6] / str(i)), self.hand_cam_rgb_array[i])
      cv2.imwrite(str(self.exp_dir / self.dir_names[7] / str(i)), self.hand_cam_depth_array[i])
      cv2.imwrite(str(self.exp_dir / self.dir_names[8] / str(i)), self.side_cam_rgb_array[i])


  def sync_with_upsample():
    pass

if __name__ == "__main__":
    '''create a node that synchronises all sensor sources
    save all the images into a directory, and numerical data into csv
    '''
    rospy.init_node("process_synced_data")
    
    save_dir = ""

    proc = SyncData(save_dir=save_dir, sync_slop=0.0001)

    # rospy.spin()
    while not rospy.is_shutdown():
      try:
        # print('looping')
        rospy.spin()
      except rospy.exceptions.ROSException:
        break

    proc.save_data()
