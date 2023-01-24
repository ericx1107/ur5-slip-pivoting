#! /usr/bin/env python

import rospy
from slip_manipulation.box_markers import BoxMarkers
from slip_manipulation.state_estimator import StateEstimator



if __name__ == "__main__":
    rospy.init_node("contact_estimation")

    markers = BoxMarkers()

    est = StateEstimator()

    while not rospy.is_shutdown():
        contact = est.vision_estimate_contact()

        est.estimate_contact_config(contact)