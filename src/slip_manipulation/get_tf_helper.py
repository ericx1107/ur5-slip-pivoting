import rospy
import tf2_ros
# import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import tf

# def tf_transform_pose(tf_buffer, input_pose, from_frame, to_frame, loop=True):
#     # make PoseStamped message from Pose input
#     pose_stamped = tf2_geometry_msgs.PoseStamped()
#     pose_stamped.pose = input_pose
#     pose_stamped.header.frame_id = from_frame
#     pose_stamped.header.stamp = rospy.Time(0)

#     if loop:
#         while not rospy.is_shutdown():
#             try:
#                 # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
#                 output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(2), PoseStamped)
#                 # tf_buffer.transform is incorrectly documented as giving no returns. The return below is mistaken by vs code as unreachable
#                 return output_pose_stamped

#             except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#                 print("Cannot transform pose\n")

#     else:
#         try:
#             # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
#             output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(2), PoseStamped)
#             # tf_buffer.transform is incorrectly documented as giving no returns. The return below is mistaken by vs code as unreachable
#             return output_pose_stamped

#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#             print("Cannot transform pose\n")

def tf_transform_pose(tf_listener, input_pose, from_frame, to_frame, loop=True):
    # make PoseStamped message from Pose input
    pose_stamped = PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    if loop:
        while not rospy.is_shutdown():
            try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                output_pose_stamped = tf_listener.transformPose(to_frame, pose_stamped)
                # tf_buffer.transform is incorrectly documented as giving no returns. The return below is mistaken by vs code as unreachable
                return output_pose_stamped

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
                print("Cannot transform pose\n")

    else:
        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                output_pose_stamped = tf_listener.transformPose(to_frame, pose_stamped)
                # tf_buffer.transform is incorrectly documented as giving no returns. The return below is mistaken by vs code as unreachable
                return output_pose_stamped

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Cannot transform pose\n")

def patient_lookup_box_tf(tf_buffer, loop=True):
    if loop:
        while not rospy.is_shutdown():
            try:
                trans = tf_buffer.lookup_transform('box_origin', 'base_link', rospy.Time(0), timeout=rospy.Duration(2))
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Waiting for box transform\n")
    else:
            try:
                trans = tf_buffer.lookup_transform('box_origin', 'base_link', rospy.Time(0), timeout=rospy.Duration(2))
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Waiting for box transform\n")

def patient_lookup_tf(tf_buffer, target_frame, loop=True):
    if loop:
        while not rospy.is_shutdown():
            try:
                trans = tf_buffer.lookup_transform(target_frame, 'base_link', rospy.Time(0), timeout=rospy.Duration(2))
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Waiting for box transform\n")
    else:
            try:
                trans = tf_buffer.lookup_transform(target_frame, 'base_link', rospy.Time(0), timeout=rospy.Duration(2))
                return trans
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Waiting for box transform\n")


                