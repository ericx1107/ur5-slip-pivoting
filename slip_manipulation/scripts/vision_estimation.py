#! /usr/bin/env python

import rospy
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.state_estimator import StateEstimator

if __name__ == "__main__":
    rospy.init_node("vision_estimator")
        
    # box_dim = [0.18, 0.11, 0.04]  # small box
    # box_dim = [0.23, 0.16, 0.05]  # square box
    box_dim = [0.28, 0.12, 0.05]  # long box
    
    # init box markers to run this node on its own
    # comment out if running alongside pivoting scripts
    # markers = BoxMarkers(box_dim)
    
    est = StateEstimator(box_dim)
    
    while not rospy.is_shutdown():
        # contact = est.vision_estimate_contact()
        
        # est.vision_estimate_contact_config(contact)

        est.vision_estimate_rotation_angle()
        
        est.vision_estimate_lowest_vertex()
    