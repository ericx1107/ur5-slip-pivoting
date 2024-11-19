import numpy as np
import copy
import matplotlib as mpl
from matplotlib import pyplot as plt

base_dir = "/home/acrv/trajectory_ws/data/tool0_ros_time_z/"

# prefix = "_use_server_time_true" + ".npy"
prefix = "_2024_11_19__07_23_56" + ".npy"

object_dist = np.load(base_dir + "obj_dist" + prefix)
object_t = np.load(base_dir + "obj_time" + prefix)
robot_dist = np.load(base_dir + "rob_dist" + prefix)
robot_t = np.load(base_dir + "rob_time" + prefix)

# object_dist_diff = np.diff(object_dist)
# object_t_diff = np.diff(object_t)
# robot_dist_diff = np.diff(robot_dist)
# robot_t_diff = np.diff(robot_t)

object_dist_diff = object_dist - object_dist[0]
object_t_diff = object_t - object_t[0]
robot_dist_diff = robot_dist - robot_dist[0]
robot_t_diff = robot_t - robot_t[0]

# plot the diffs
fig = plt.figure()
ax1 = fig.add_subplot(111)
ax1.scatter(object_t_diff*1e-9, object_dist_diff, s=(mpl.rcParams['lines.markersize'] ** 2)/8)
ax1.scatter(robot_t_diff*1e-9, robot_dist_diff, s=(mpl.rcParams['lines.markersize'] ** 2)/8, c='r')
ax1.set_xlabel('Time')
ax1.set_ylabel('change in object distance from origin')

plt.show()

print(object_dist.shape)
print(object_dist_diff.shape)
# print(object_dist_diff)

threshold = 1e-03
index = np.argmax(abs(object_dist_diff) > threshold)

print('start index of change in object distance:')
print(index, object_dist_diff[index-1], object_dist_diff[index], object_dist_diff[index+1])
index = np.argmax(abs(robot_dist_diff) > threshold)
print('start index of change in robot distance:')
print(index, robot_dist_diff[index-1], robot_dist_diff[index], robot_dist_diff[index+1])

print('change in time since start for object:')
print(object_t_diff[index])
print('change in time since start for robot:')
print(robot_t_diff[index])

object_dist_end_diff = object_dist - object_dist[-1]
object_t_end_diff = object_t - object_t[-1]
robot_dist_end_diff = robot_dist - robot_dist[-1]
robot_t_end_diff = robot_t - robot_t[-1]
# print(robot_dist_end_diff)

threshold = 1e-04
end_index = np.argmax(abs(object_dist_end_diff) < threshold)

print('end index of change in object distance:')
print(end_index, object_dist_diff[end_index-1], object_dist_diff[end_index], object_dist_diff[end_index+1])
end_index = np.argmax(abs(robot_dist_end_diff) < threshold)
print('end index of change in robot distance:')
print(end_index, robot_dist_diff[end_index-1], robot_dist_diff[end_index], robot_dist_diff[end_index+1])

print('change in time since start for object:')
print(object_t_diff[end_index])
print('change in time since start for robot:')
print(robot_t_diff[end_index])

print('duration time object:')
print(object_t_diff[end_index] - object_t_diff[index])
print('duration time robot:')
print(robot_t_diff[end_index] - robot_t_diff[index])
print('duration time difference (s):')
print(abs((object_t_diff[end_index] - object_t_diff[index]) - (robot_t_diff[end_index] - robot_t_diff[index]))*1e-9)

print('start time object:')
print(object_t[index])
print('start time robot:')
print(robot_t[index])
print('start time difference (s):')
print(abs(object_t[index] - robot_t[index])*1e-9)