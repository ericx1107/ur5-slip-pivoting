#! /usr/bin/env python

import rospy
import numpy as np
import open3d as o3d
import time
from sensor_msgs.msg import PointCloud2
from open3d_ros_helper import rospc_to_o3dpc, o3dpc_to_rospc
from sensor_msgs.msg import PointCloud2

class D405Camera():
    def __init__(self):
        self.original_pcd_topic = '/camera/depth/color/points'
        self.seg_pcd_topic = self.original_pcd_topic + '_processed'

        self.point_cloud_listener = rospy.Subscriber(
            self.original_pcd_topic,
            PointCloud2,
            self.callback,
            queue_size=1,
        )

        self.point_cloud_publisher = rospy.Publisher(
            self.seg_pcd_topic,
            PointCloud2,
            queue_size=1
        )


    def callback(self, pcd_ros):

        start_time = time.time()

        pcd_frame = pcd_ros.header.frame_id
        # print(pcd_frame)

        pcd = rospc_to_o3dpc(pcd_ros)
        print('src:', pcd)

        # o3d.io.write_point_cloud("raw.pcd", pcd, print_progress=True)

        # Remove points further away than this threshold
        distance_threshold = 1.2

        #norms = np.linalg.norm(np.asarray(pcd.points), axis=np.argmin(np.asarray(pcd.points).shape))
        dist = np.asarray(pcd.points)[:, 0]

        pcd = pcd.select_by_index([i for i in range(len(dist)) if dist[i] < distance_threshold])
        print('remove far points:', pcd)

        # Remove points lower than this threshold
        height_threshold = 0.8

        heights = np.asarray(pcd.points)[:, 2]

        pcd = pcd.select_by_index([i for i in range(len(heights)) if heights[i] > height_threshold])
        print('remove low points:', pcd)

        # Remove planes with more points than this threshold
        plane_threshold = 10000

        for i in range(5):
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=50)
            plane_cloud = pcd.select_by_index(inliers)         # plane
            plane_size = len(plane_cloud.points)

            if plane_size < plane_threshold:
                continue
            
            
            pcd = pcd.select_by_index(inliers, invert=True)       # rest of the objects

            # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
            # print(f"Removed plane with {plane_size} points")
        
        # Clean up point cloud
        # pcd, ind = pcd.remove_radius_outlier(nb_points=500, radius=0.04)
        print('remove radius outliers:', pcd)

        # Publish segmented point cloud
        pcd_msg = o3dpc_to_rospc(pcd, frame_id=pcd_frame)

        self.point_cloud_publisher.publish(pcd_msg)
        print("Publishing objects point cloud...")

        # print(f"Time taken: {time.time() - start_time}")