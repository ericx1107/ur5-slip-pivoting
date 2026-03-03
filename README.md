# UR5 Slip In-Hand Pivoting
ROS package for in-hand pivoting experiments with a UR5 robot arm. The package was created and tested on ROS melodic and Ubuntu 18.04.

## Building

```bash
# clone to src folder in ros workspace
git clone https://github.com/exsyu/ur5-slip-pivoting.git src/
```

For Python3 components (especially related to OpenCV), build with the following:
```bash
# replace python3.6m with actual python version
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE="$(which python)" -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```
But a lot of packages actually use pyhon2 syntax (e.g. robotiq 2f stuff), so actually build with:
```bash
catkin_make
```

For the robot specific roslaunches, use normal python2.

For the digit related things, use python3 (Python 3.6.9 used).

Best to do this in a virtual environment, for example, using venv:
```bash
# creat venv using Python 3.6.9
python3 -m venv traj

. ./traj/bin/activate

pip install -r requirements.txt
```


We use slip in the gripper fingers to enable the rotation of a target box object, while using multiple data-modalities to track the state of the object. For details see [our paper](https://ieeexplore.ieee.org/abstract/document/10341505). 


## Equipment and Sensors
- [UR5 robot arm](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master)[with MoveIt!](https://github.com/ros-industrial/universal_robot)
- [RobotiQ 2F-85 gripper](https://wiki.ros.org/robotiq_2f_gripper_control)
- [RobotiQ FT-300 force torque sensor](https://wiki.ros.org/robotiq_ft_sensor)
- [Contactile PapillArray 3x3 sensor array](https://contactile.com/products/#tactile_sensors)
- [Realsense D435i camera on the side](https://wiki.ros.org/realsense2_camera)

## Files
Python class modules are stored in the `src` directory, scripts to be run are stored in the `scripts` directory.

The ROS launch files are `launch` directory, the launchfile `setup_system.launch` sets up the robot and sensors for operating, and the `record-*.launch` files are used for data collection in experiments.

A simple custom message is in the `msg` directory.

The `config` directory contains some useful rviz presets.

## Useful commands
### Zero FT 300 sensor
```
rosservice call /robotiq_ft_sensor_acc "command_id: 8"

# zero tactile sensors
rosservice call /hub_0/send_bias_request
```

### run the simple trajectory data collection
```
roslaunch vistac_trajectory simple_trajectory.launch
```