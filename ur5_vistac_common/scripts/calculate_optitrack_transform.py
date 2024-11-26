#! /usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import os
import datetime as dt
from datetime import datetime
from pathlib import Path
from geometry_msgs.msg import WrenchStamped, PoseStamped
from scipy.spatial.transform import Rotation as R
from tf import TransformListener

class CalibrateOptitrackFrame():
  '''To calibrate the optitrack system's coordinate system and get the transform to the robot's base frame
  '''
  def __init__(self, save_dir, sync_slop=0.0001):
    # init data arrays
    self.oTe = [] # o for optitrack origin, e for robot end effector, b for robot base
    self.bTe = []
    self.ote = []
    self.bte = []
    self.time_array = []
    
    # init ros subscribers
    # self.mocap_sub = message_filters.Subscriber('/tool0/pose', PoseStamped)
    # self.base_sub = message_filters.Subscriber('')
    self.mocap_sub = rospy.Subscriber('/tool0/pose', PoseStamped, callback=self.mocap_callback)
    
    self.tf_listener = TransformListener()
    
    # init save paths
    # make a folder for the experiment
    t = datetime.now(dt.timezone(dt.timedelta(hours=10)))
    dt_string = t.strftime("%Y_%m_%d___%H_%M_%S")
    self.exp_dir = save_dir / Path(dt_string)
    os.mkdir(self.exp_dir)   # do not allow same name directories
    
    # # set up message filter synchroniser
    # self.ts = message_filters.ApproximateTimeSynchronizer(
    #   [self.mocap_sub, self.], 
    #   queue_size=10, slop=sync_slop)
    # self.ts.registerCallback(self.ts_callback)
    

  def mocap_callback(self, data):
    # get the position and orientation and convert to a transformation matrix
    po = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    ro = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    To = R.from_quat(ro)
    To[:-1, -1] = po
    
    # get the eef pose data from the robot base frame
    # self.tf_listener.waitForTransform('/base', '/tool0', rospy.Time(), timeout=rospy.Duration(0.2))
    (pb, rb) = self.tf_listener.lookupTransform('/base', '/tool0', rospy.Time(0))

    
    # convert eef pose in base frame to transformation matrix
    Tb = R.from_quat(rb)
    Tb[:-1, -1] = pb
    
    # append to data arrays
    self.oTe.append(To)
    self.bTe.append(Tb)
    self.ote.append(po)
    self.bte.append(pb)
    self.time_array.append(data.header.stamp.to_nsec())

  # def ts_callback(self,)
  def timer_callback(self):
    rospy.loginfo("10 seconds elapsed. Stopping spin.")
    rospy.signal_shutdown("Timer expired.")

if __name__ == "__main__":
  save_dir = ""
  c = CalibrateOptitrackFrame(save_dir=save_dir)
  
  # collect 10 seconds worth of data
  rospy.Timer(rospy.Duration(10), c.timer_callback, oneshot=True)
  
  rospy.loginfo("Spin for 10 seconds")
  rospy.spin()
  rospy.loginfo("Collating data")
  
  mean_bte = np.mean(np.array(c.bte), axis=0)
  mean_ote = np.mean(np.array(c.ote), axis=0)
  
  print(mean_ote - mean_bte)
  