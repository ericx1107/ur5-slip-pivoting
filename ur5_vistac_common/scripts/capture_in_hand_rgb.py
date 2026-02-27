#! /usr/bin/env python

import rospy
import message_filters
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import os
import cv2
import cv_bridge
from datetime import datetime, tzinfo, timedelta
from pathlib import Path
from matplotlib.lines import Line2D
from slip_manipulation.msg import AngleStamped
from geometry_msgs.msg import WrenchStamped, PoseStamped
from papillarray_ros_v2.msg import SensorState
from sensor_msgs.msg import Image, CompressedImage
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

    self.pose_array = []

    self.time_array = []
    self.dt_array = []
    
    self.bridge = cv_bridge.CvBridge()

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
    dt_string = t.strftime("%Y_%m_%d___%H_%M_ %S")
    
    self.exp_dir = save_dir / Path(dt_string)
    os.mkdir(str(self.exp_dir))   # do not allow same name directories
    # make a folder for the individual modalities
    self.dir_names = ['in_hand_rgb', 'pose']
    for dir in self.dir_names:
      os.makedirs(str(self.exp_dir / dir))

    # subscribers
    # self.wrench_sub = message_filters.Subscriber('/robotiq_ft_wrench', WrenchStamped) # 60hz
    # self.tac0_sub = message_filters.Subscriber('/hub_0/sensor_0', SensorState)  # 500hz
    # self.tac1_sub = message_filters.Subscriber('/hub_0/sensor_1', SensorState)
    self.hand_cam_rgb_sub = message_filters.Subscriber('/d405/color/image_raw', Image)  # 60hz
    # self.hand_cam_rgb_sub = message_filters.Subscriber('/d405/color/image_raw/compressed', CompressedImage)
    # self.hand_cam_depth_sub = message_filters.Subscriber('/d405/depth/image_rect_raw', Image) # 60hz
    # self.hand_cam_depth_sub = message_filters.Subscriber('/d405/aligned_depth_to_color/image_raw', Image) # 60hz
    # self.hand_cam_depth_sub = message_filters.Subscriber('/d405/depth/image_rect_raw/compressed', CompressedImage)
    # self.side_cam_rgb_sub = message_filters.Subscriber('/d435/color/image_raw', Image)  # 60hz
    self.mocap_obj_sub = message_filters.Subscriber('vrpn_client_node/Scrub/pose', PoseStamped)

    self.ts = message_filters.ApproximateTimeSynchronizer(
      [self.hand_cam_rgb_sub, 
       self.mocap_obj_sub], 
      queue_size=10, slop=sync_slop)
    self.ts.registerCallback(self.callback)

  def callback(self, hand_rgb, pose):
    print("caught frames")
    # process in hand camera images
    # save into big numpy array/list and loop through later? might be too large to do live
    self.hand_cam_rgb_array.append(hand_rgb)



    # process object poses [posx, posy, posz, quatx, quaty, quatz, quatw] 
    self.pose_array.append([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 
                            pose.pose.orientation.x, pose.pose.orientation.y, 
                            pose.pose.orientation.z, pose.pose.orientation.w])
    

  def save_data(self):
    # loop through each frame
    print("\n\nSaving data")
    print("Total frames: {}".format(len(self.hand_cam_rgb_array)))
    for i in range(len(self.hand_cam_rgb_array)):
    
      # save camera images
      # save the image as rgb or binary
      cv2.imwrite(str(self.exp_dir / self.dir_names[0] / (str(i) + ".png")), 
                  cv2.cvtColor(
                    self.bridge.imgmsg_to_cv2(self.hand_cam_rgb_array[i]), cv2.COLOR_RGB2BGR
                    ))
      
      # save object poses
      np.save(self.exp_dir / self.dir_names[1] / str(i), self.pose_array[i])
      
    print("Finished save")
    print("dt average: {0:.2f}; standard deviation: {1:.2f}".format(np.mean(self.dt_array), np.std(self.dt_array)))


  def sync_with_upsample():
    pass

if __name__ == "__main__":
    '''create a node that synchronises all sensor sources
    save all the images into a directory, and numerical data into csv
    '''
    rospy.init_node("capture_in_hand_rgb")
    
    save_dir = "/home/acrv/trajectory_ws/data/experiments"

    proc = SyncData(save_dir=save_dir, sync_slop=0.5)

    # rospy.spin()
    while not rospy.is_shutdown():
      try:
        # print('looping')
        rospy.spin()
      except rospy.exceptions.ROSException:
        break

    rospy.on_shutdown(proc.save_data)
