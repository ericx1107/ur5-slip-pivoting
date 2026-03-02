#! /usr/bin/env python3

"""
for capturing individual frames of depth images from the wrist camera
"""

# import rospy
# from ur5_vistac_common.d405_camera import D405ForKeypoints

# if __name__ == "__main__":
#     rospy.init_node('capture_individual_depth_frame')
    
#     d = D405ForKeypoints()
    
#     rospy.spin()
    
import rospy
from geometry_msgs.msg import PoseStamped
import sensor_msgs.point_cloud2 as pc2
# import pcl
# import pcl.pcl_visualization
import open3d as o3d
import numpy as np
import struct
import argparse
from pathlib import Path
from datetime import datetime, tzinfo, timedelta
import os
import cv_bridge
from PIL import Image as PILImage

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

import copy
from sensor_msgs.msg import PointCloud2, PointField, Image
import ros_numpy
import colorsys  # For RGB to HSV conversion

from digit_interface import Digit
import cv2
import numpy as np
import time
import os
from cv_bridge import CvBridge
from collections import deque, defaultdict

# def init_digits():
#     # Initialize the DIGIT device
#     dl = Digit("D20233")  # Replace with your sensor's serial number or use Digit("")
#     dr = Digit("D20237")

#     dl.connect()
#     dl.set_resolution(Digit.STREAMS["VGA"])
#     dl.set_fps(Digit.STREAMS["VGA"]["fps"]["15fps"])

#     dr.connect()
#     dr.set_resolution(Digit.STREAMS["VGA"])
#     dr.set_fps(Digit.STREAMS["VGA"]["fps"]["15fps"])
    
#     return dl, dr

# def disconnect_digits(digits_list):
#     for d in digits_list:
#         d.disconnect()

# def get_digit_frames(dl, dr):
#     lframe = dl.get_frame()  # This returns a BGR image as a NumPy array
#     rframe = dr.get_frame()  # This returns a BGR image as a NumPy array
    
#     return lframe, rframe

def rgb_to_hsv(r, g, b):
    """Convert RGB to HSV."""
    r, g, b = r / 255.0, g / 255.0, b / 255.0  # Normalize to [0, 1] range
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    return h, s, v

def img_msg_to_img(msg, save_dir, format="JPEG"):
    # Get the image data from the ROS message
    img_data = np.frombuffer(msg.data, dtype=np.uint8)  # Convert raw byte data to NumPy array
    
    # Reshape the image data based on the dimensions and encoding of the image
    # For example, if the image is in RGB format:
    height = msg.height
    width = msg.width
    encoding = msg.encoding
    
    if encoding == "rgb8":
        img_data = img_data.reshape((height, width, 3))  # For RGB images
    elif encoding == "bgr8":
        img_data = img_data.reshape((height, width, 3))  # For BGR images
    elif encoding == "mono8":
        img_data = img_data.reshape((height, width))  # For grayscale images
    else:
        rospy.logerr("Unsupported encoding: %s", encoding)
        return
    
    # Convert the NumPy array to a Pillow Image
    pil_img = PILImage.fromarray(img_data)
    
    # Save the image as a PNG file
    pil_img.save(save_dir, format)

def o3dpc_to_rospc(o3dpc, frame_id=None, stamp=None):
    """ from python package open3d-ros-helper, importing this thing is a cunt
    """
    """ convert open3d point cloud to ros point cloud
    Args:
        o3dpc (open3d.geometry.PointCloud): open3d point cloud
        frame_id (string): frame id of ros point cloud header
        stamp (rospy.Time): time stamp of ros point cloud header
    Returns:
        rospc (sensor.msg.PointCloud2): ros point cloud message
    """

    cloud_npy = np.asarray(copy.deepcopy(o3dpc.points))
    is_color = o3dpc.colors
        

    n_points = len(cloud_npy[:, 0])
    if is_color:
        data = np.zeros(n_points, dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('rgb', np.uint32)
        ])
    else:
        data = np.zeros(n_points, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
            ])
    data['x'] = cloud_npy[:, 0]
    data['y'] = cloud_npy[:, 1]
    data['z'] = cloud_npy[:, 2]
    
    if is_color:
        rgb_npy = np.asarray(copy.deepcopy(o3dpc.colors))
        rgb_npy = np.floor(rgb_npy*255) # nx3 matrix
        rgb_npy = rgb_npy[:, 0] * BIT_MOVE_16 + rgb_npy[:, 1] * BIT_MOVE_8 + rgb_npy[:, 2]  
        rgb_npy = rgb_npy.astype(np.uint32)
        data['rgb'] = rgb_npy

    rospc = ros_numpy.msgify(PointCloud2, data)
    if frame_id is not None:
        rospc.header.frame_id = frame_id

    if stamp is None:
        rospc.header.stamp = rospy.Time.now()
    else:
        rospc.header.stamp = stamp
    rospc.height = 1
    rospc.width = n_points
    rospc.fields = []
    rospc.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    rospc.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    rospc.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))    

    if is_color:
        rospc.fields.append(PointField(
                        name="rgb",
                        offset=12,
                        datatype=PointField.UINT32, count=1))    
        rospc.point_step = 16
    else:
        rospc.point_step = 12
    
    rospc.is_bigendian = False
    rospc.row_step = rospc.point_step * n_points
    rospc.is_dense = True
    return rospc

def process_points(point_cloud_msg):
    """
    colour based points segmentation for red mug object
    """
    # Convert PointCloud2 to XYZRGB format
    cloud = list(pc2.read_points(point_cloud_msg, field_names=["x", "y", "z", "rgb"], skip_nans=True))

    points = []

    for p in cloud:
        x, y, z, rgb = p
        
        # Compute the Euclidean distance from the origin (0, 0, 0)
        distance = np.sqrt(x**2 + y**2 + z**2)

        # Exclude points that are farther than 50 cm (0.5 meters)
        if distance > 0.5:
            continue  # Skip this point if it's too far
        
        # Convert the RGB float to an integer using struct.unpack
        # rgb is a float in PointCloud2, so we need to interpret it as a 32-bit integer
        rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
        
        r = (rgb_int >> 16) & 0x0000ff
        g = (rgb_int >> 8) & 0x0000ff
        b = (rgb_int) & 0x0000ff

        # Convert RGB to HSV
        h, s, v = rgb_to_hsv(r, g, b)

        # Define HSV thresholds to filter (example: keep only highly saturated, bright colors)
        if (0.0 <= h <= 0.2 or 0.8 <= h <= 1) and 0.5 <= s <= 1.0 and 0.3 <= v <= 1.0:  # Filter for red colors
            points.append([x, y, z, r, g, b])
            
    # Now you have segmented red points in `points`
    print(f"Segmented {len(points)} red points")

    # Convert the points list to numpy arrays
    xyz_points = np.array([[p[0], p[1], p[2]] for p in points])

    # Create a PointCloud object in Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_points)

    # Extract RGB colors for the points and normalize them to [0, 1]
    colors = np.array([[p[3] / 255.0, p[4] / 255.0, p[5] / 255.0] for p in points])

    # Set the colors of the points in the PointCloud object
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Convert Open3D PointCloud to PointCloud2
    # ros_pc = create_point_cloud2_from_o3d(pcd)
    ros_pc = o3dpc_to_rospc(pcd)
    ros_pc.header.frame_id = point_cloud_msg.header.frame_id
            
    # np array, o3d point cloud, ros message
    return xyz_points, pcd, ros_pc

def one_message(save_path):
    rospy.init_node('color_segmentation_node', anonymous=True)
    point_cloud_msg = rospy.wait_for_message("/d405/depth/color/points", PointCloud2)
    
    # segment the points in XYZRGB format
    # xyz_points, pcd, ros_pc = process_points(point_cloud_msg)

    # save the point cloud as npy files
    if save_path is not None:        
        # save the depth points (both numpy and o3d)
        # depth_path = save_path / "in_hand_depth"
        # np.save(depth_path / f"partial_depth.npy", xyz_points)
        # o3d.io.write_point_cloud(str(depth_path / f"partial_depth.ply"), pcd)
        
        # save the digit frames
        digit_path = save_path / "digits"
        left_msg = rospy.wait_for_message("/digit/image_raw/left", Image)
        right_msg = rospy.wait_for_message("/digit/image_raw/right", Image)
        
        # bridge = CvBridge()
        # lframe = bridge.imgmsg_to_cv2(left_msg)
        # rframe = bridge.imgmsg_to_cv2(right_msg)
        
        # cv2.imwrite(str(digit_path / f"left.png"), lframe)
        # cv2.imwrite(str(digit_path / f"right.png"), rframe)
        
        img_msg_to_img(left_msg, save_dir=str(digit_path / f"left.png"), format="PNG")
        img_msg_to_img(right_msg, save_dir=str(digit_path / f"right.png"), format="PNG")
        
        
        # save the mocap pose
        # pose_path = save_path / "pose"
        # pose_msg = rospy.wait_for_message('vrpn_client_node/RedMug/pose', PoseStamped)
        # pose_array = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, 
        #                     pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, 
        #                     pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])
        # np.save(pose_path / f"obj_pose.npy", pose_array)
        
        # save the rgb image (compressed)
        rgb_path = save_path / "in_hand_rgb"
        rgb_msg = rospy.wait_for_message('/d405/color/image_raw', Image)
        # save the image as rgb or binary
        # bridge = cv_bridge.CvBridge()
        # cv2.imwrite(str(rgb_path / "rgb.jpg"), 
        #           cv2.cvtColor(
        #             bridge.imgmsg_to_cv2(rgb_msg), cv2.COLOR_RGB2BGR
        #             ))
        img_msg_to_img(rgb_msg, save_dir=str(rgb_path / "rgb.jpg"))
        
        print(f"Finished saving to {save_path}")
        

class KeypointsData():
    def __init__(self, save_path=None,
                 cam_only=False, 
                 voxel_size=0.003,          # 5 mm voxels (tune)
                 window_frames=60,          # ~1s depth camera at 60Hz (tune)
                 min_fraction=0.8):         # must appear in >=60% of frames
        self.cam_only = cam_only
        self.i = 0
                
        rospy.init_node('color_segmentation_node', anonymous=True)
        self.publisher = rospy.Publisher("/d405/depth/color/points_processed", PointCloud2, queue_size=1)
        self.publisher_avg = rospy.Publisher("/d405/depth/color/points_averaged", PointCloud2, queue_size=1)
        
        self.save_path = save_path
        
        self.bridge = CvBridge()
        
        # --- temporal voxel accumulator ---
        self.voxel_size = float(voxel_size)
        self.window_frames = int(window_frames)
        self.min_fraction = float(min_fraction)

        # Each entry in deque is (keys, xyz, counts_per_point) for that frame
        # We store point-level contributions so we can subtract them when the frame leaves the window.
        self._frame_queue = deque(maxlen=self.window_frames)

        # Global accumulators over the sliding window
        self._count = defaultdict(int)  # key -> int
        self._sum = defaultdict(lambda: np.zeros(3, dtype=np.float64))  # key -> (3,)

    def callback(self, point_cloud_msg):
        # segment the points in XYZRGB format
        xyz_points, pcd, ros_pc = process_points(point_cloud_msg)
        
        # --- voxel accumulator ---
        vidx = self.voxel_indices(xyz_points, self.voxel_size)
        keys = self.pack_voxel_keys(vidx)
        
        # add contributions for this frame
        for k, p in zip(keys, xyz_points):
            self._count[k] += 1
            self._sum[k] += p.astype(np.float64)
            
        # push into sliding-window queue
        self._frame_queue.append((keys, xyz_points))
        
        # If deque is full and an old frame was evicted, subtract it manually.
        # deque(maxlen=...) discards silently, so we handle overflow ourselves:
        while len(self._frame_queue) > self.window_frames:
            old_keys, old_xyz = self._frame_queue.popleft()
            for k, p in zip(old_keys, old_xyz):
                self._count[k] -= 1
                self._sum[k] -= p.astype(np.float64)
                if self._count[k] <= 0:
                    del self._count[k]
                    del self._sum[k]

        # wait until there is enough history before publishing averaged pc
        # manual tune for required history length
        if len(self._frame_queue) < max(3, int(0.7 * self.window_frames)):
            # publish raw until it fills
            self.publisher.publish(ros_pc)
            return
        
        # --- build averaged pc from frequent voxels ---
        # only keep voxels that appear in more than the min_fraction of recent frames
        min_count = int(np.ceil(self.min_fraction * len(self._frame_queue)))
        out_pts = []
        for k, c in self._count.items():
            if c >= min_count:
                # calculate the average of all accumulated points
                out_pts.append(self._sum[k] / float(c))
        
        if len(out_pts) == 0:
            # fallback to original pc
            print(f"[WARN] No voxel has been populated \
                for more than {self.min_fraction*100}% over the window")
            
        out_pts = np.asarray(out_pts, dtype=np.float32)
        
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(out_pts)
        
        # paint red for visuals
        out_colors = np.tile(np.array([[1.0, 0.0, 0.0]], dtype=np.float32), (out_pts.shape[0], 1))
        out_pcd.colors = o3d.utility.Vector3dVector(out_colors)

        out_ros = o3dpc_to_rospc(out_pcd)
        out_ros.header.frame_id = point_cloud_msg.header.frame_id
        out_ros.header.stamp = point_cloud_msg.header.stamp
        
        # save the processed point cloud as npy files
        # random sampler to decide whether to save
        # 20 frames per second
        if self.save_path is not None and np.random.random() > 20/60:
            start_t = time.time()
            
            # save the depth points (both numpy and o3d)
            depth_path = self.save_path / "in_hand_depth"
            np.save(depth_path / f"partial_depth_{self.i}.npy", xyz_points)
            o3d.io.write_point_cloud(str(depth_path / f"partial_depth_{self.i}.ply"), pcd)
            # save the averaged depth points
            np.save(depth_path / f"partial_depth_{self.i}_averaged.npy", out_pts)
            o3d.io.write_point_cloud(str(depth_path / f"partial_depth_{self.i}_averaged.ply"), out_pcd)
            
            if not self.cam_only:
                # save the digit frames
                digit_path = self.save_path / "digits"
                left_msg = None
                right_msg = None
                while left_msg == None or right_msg == None:
                    digit_msg = rospy.wait_for_message("/digit/image_raw", Image)
                    if "left" in digit_msg.header.frame_id:
                        left_msg = digit_msg
                    elif "right" in digit_msg.header.frame_id:
                        right_msg = digit_msg
                    else:
                        print("wot")
                
                lframe = self.bridge.imgmsg_to_cv2(left_msg)
                rframe = self.bridge.imgmsg_to_cv2(right_msg)
                
                cv2.imwrite(str(digit_path / f"left_{self.i}.png"), lframe)
                cv2.imwrite(str(digit_path / f"right_{self.i}.png"), rframe)
            
                # save the mocap pose
                pose_path = self.save_path / "pose"
                #TODO: finish this
            
            self.i += 1
            
            # save the rgb image (compressed)
            rgb_path = self.save_path / "in_hand_rgb"
            
        self.publisher.publish(ros_pc)
        print(f"Averaged {out_pts.shape[0]} points")
        self.publisher_avg.publish(out_ros)
        

    def listener(self):
        rospy.Subscriber("/d405/depth/color/points", PointCloud2, self.callback)
        rospy.spin()

    @staticmethod
    def voxel_indices(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
        """
        split a size N set of points into voxels
        xyz: (N,3) float
        returns: (N,3) int voxel coords
        """
        return np.floor(xyz / voxel_size).astype(np.int32)

    @staticmethod
    def pack_voxel_keys(vidx: np.ndarray) -> np.ndarray:
        """
        Pack int32 (N,3) voxel coords into int64 keys for dict usage.
        Safe for reasonable ranges.
        """
        vx = vidx[:, 0].astype(np.int64)
        vy = vidx[:, 1].astype(np.int64)
        vz = vidx[:, 2].astype(np.int64)
        # simple bijection-like packing (assumes coords not astronomically large)
        return (vx << 42) ^ (vy << 21) ^ vz
    
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', action='store_true', help='')
    parser.add_argument('--spin', action='store_true')
    parser.add_argument('--cam-only', action='store_true', help='skip digit saving')
    args = parser.parse_args()
    
    if args.save:
        save_root = Path("/home/acrv/trajectory_ws/keypoints_data/")
        # make a folder with the current time
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
        
        exp_dir = save_root / Path(dt_string)
        os.mkdir(str(exp_dir))   # do not allow same name directories
        # make a folder for the individual modalities
        dir_names = ['in_hand_rgb', 'pose', 'in_hand_depth', 'digits']
        for dir in dir_names:
            os.makedirs(str(exp_dir / dir))
    else:
        exp_dir = None
    
    if args.spin:
        k = KeypointsData(save_path=exp_dir, cam_only=args.cam_only)
        k.listener()
    else:
        one_message(save_path=exp_dir)