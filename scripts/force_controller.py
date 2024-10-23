import rospy
import numpy as np
from robotiq_ft_sensor.msg import ft_sensor
from slip_manipulation.msg import AngleStamped
from slip_manipulation.arc_trajectory import ArcTrajectory

fts_data = []
theta = 0
Fz = 0

def ftSensor_callback(data):
    fts_data = data
    Fz = data.Fz

def angle_callback(data):
    theta = np.radians(data)
    
def force_pred(theta):
    if 0 <= theta and theta < phi:
        force = -0.6 * fg * np.log(theta + (1 - phi))
    elif phi <= theta and theta <= np.pi/2:
        force = -0.25 * fg * (theta - phi)**3 * (theta - np.pi/2) * (theta + 11)
    return force

fts_sub = rospy.Subscriber('/robotiq_ft_wrench', ft_sensor, ftSensor_callback)
angle_sub = rospy.Subscriber('/slip_manipulation/rotation_angle', AngleStamped, angle_callback)

h = 0.11
l = 0.18
box_dim = [h,l]
weight = 1.276
fg = 9.8 * weight
phi = np.arctan(h/l)


    
arc = ArcTrajectory(box_dim, self.ur5.arm, self.grasp_param)
waypoints = arc.plan_cartesian_path()

i = 0
temp = []
while waypoints != []:
    for i in range(5):
        temp.append(waypoints[i])
        waypoints.pop(0)
    plan = arc.arm.compute_cartesian_path(temp, 0.01, 0.0)
    demo.arm.execute(cartesian_plan, wait=True)
    predicted_force = force_pred(theta)
    
    