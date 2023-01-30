#! /usr/bin/env python

import rospy
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.state_estimator import StateEstimator

if __name__ == "__main__":
    rospy.init_node("vision_estimator")
        
    box_dim = [0.18, 0.11, 0.04]
    
    # markers = BoxMarkers(box_dim, 1)
    
    est = StateEstimator(box_dim)
    
    while not rospy.is_shutdown():
        # contact = est.vision_estimate_contact()
        
        # est.estimate_contact_config(contact)

        est.estimate_rotation_angle()
    
    # rospy.spin()