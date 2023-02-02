# Box rotation with slip in parallel gripper

## Current assumptions
- Box dimensions known
- Marker transform to box origin hard-coded
- Can see marker at the start of and during manipulation
- All grasps are top down
- Restricted direction of pivont once grasped

# Instructions to get [Armer](https://github.com/qcr/armer) working
## Get Python Robotics toolbox
1. [Install Python 3.8](https://askubuntu.com/questions/1197683/how-do-i-install-python-3-8-in-lubuntu-18-04) (or any other compatible version)
2. Make a virtural environment
3. Run `pip install roboticstoolbox-python` in the venv, ensuring pip resolves to the correct version of Python
4. May need to get other pre-req python packages first (e.g. [setuptools](https://stackoverflow.com/questions/11425106/python-pip-install-fails-invalid-command-egg-info))

## Get armer
1. Install [Armer drivers](https://github.com/qcr/armer) and [UR5 drivers](https://github.com/qcr/armer_ur)
2. Remember to `rosdep` for all dependencies
3. Build with python3.8 directories ([original ROS answers post](https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/)):
```bash
# or catkin build
catkin_make --cmake-args \ 
    -DCMAKE_BUILD_TYPE=Release \ 
    -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \ 
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \ 
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so

# in one line
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3.8 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so
```
4. Source
5. Can try Swift simulation by running:
```bash
roslaunch armer armer.launch config:={PATH_TO_CONFIG_YAML_FILE}
```
6. For more instructions and tutorials visit [Armer documentations](https://open.qcr.ai/armer/index.html)

# Command to zero FT 300 sensor
```
rosservice call /robotiq_ft_sensor_acc "command_id: 8"
```