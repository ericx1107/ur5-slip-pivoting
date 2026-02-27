#!/usr/bin/env python

"""
publisher for the images from the digit sensors
"""

import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import digit_interface
from digit_interface import Digit

def make_image_msg(frame, frame_id="digit_sensor"):
    """
    Build a sensor_msgs/Image from a numpy array without cv_bridge.
    Expects frame to be HxWx3 uint8 in BGR order (as returned by DIGIT).
    """
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("Expected HxWx3 image")

    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    h, w, _ = frame.shape
    msg.height = h
    msg.width = w
    msg.encoding = "bgr8"          # DIGIT returns BGR uint8 frames
    msg.is_bigendian = 0
    msg.step = w * 3               # 3 bytes per pixel
    msg.data = frame.tobytes()     # or frame.reshape(-1).tobytes()

    return msg

def manual_def_for_digits():
    # Initialize the DIGIT device
    dl = Digit("D20233")  # Replace with your sensor's serial number or use Digit("")
    dr = Digit("D20237")

    try:
        dl.disconnect()
        print("disconnected dl")
    except:
        print("no need to disconnect dl")
        pass
    try:
        dr.disconnect()
        print("disconnected dr")
    except:
        print("no need to disconnect dl")
        pass

    dl.connect()
    dl.set_resolution(Digit.STREAMS["VGA"])
    dl.set_fps(Digit.STREAMS["VGA"]["fps"]["15fps"])

    dr.connect()
    dr.set_resolution(Digit.STREAMS["VGA"])
    dr.set_fps(Digit.STREAMS["VGA"]["fps"]["15fps"])
    
    return dl, dr

def disconnect_digits(digits_list):
    for d in digits_list:
        d.disconnect()

def main():
    rospy.init_node("digit_image_publisher", anonymous=True)

    topic = rospy.get_param("~topic", "/digit/image_raw")
    rate_hz = rospy.get_param("~rate", 30)

    left_pub = rospy.Publisher(topic + "/left", Image, queue_size=10)
    right_pub = rospy.Publisher(topic + "/right", Image, queue_size=10)
    

    # Connect to first available DIGIT
    # serials = digit_interface.digit_handler.DigitHandler.list_digits()
    # if not serials:
    #     rospy.logerr("No DIGIT sensors found.")
    #     return
    # serial = serials[0]['serial']
    # digit = Digit(serial=serial)
    # digit.connect()
    # rospy.loginfo(f"Connected to DIGIT sensor {serial}")
    
    # Manually set DIGITS
    dl, dr = manual_def_for_digits()

    # bridge = CvBridge()

    rate = rospy.Rate(rate_hz)
    
    # rospy.on_shutdown(disconnect_digits([dl, dr]))
    while not rospy.is_shutdown():
        lframe = dl.get_frame()  # numpy uint8 HxWx3 (BGR)
        rframe = dl.get_frame()  # numpy uint8 HxWx3 (BGR)
        
        # left_img_msg = bridge.cv2_to_imgmsg(lframe, encoding="bgr8")
        # left_img_msg.header.stamp = rospy.Time.now()
        # left_img_msg.header.frame_id = "digit_sensor_left"
        # right_img_msg = bridge.cv2_to_imgmsg(rframe, encoding="bgr8")
        # right_img_msg.header.stamp = rospy.Time.now()
        # right_img_msg.header.frame_id = "digit_sensor_right"
        
        left_img_msg = make_image_msg(lframe, frame_id="digit_sensor_left")
        right_img_msg = make_image_msg(rframe, frame_id="digit_sensor_right")
        
        
        left_pub.publish(left_img_msg)
        right_pub.publish(right_img_msg)
        rate.sleep()
    

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
