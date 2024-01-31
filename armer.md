# Instructions to get [Armer](https://github.com/qcr/armer) working
This was ultimately not used in the project but remains an alternative way to control the robot using control techniques.

## Get Python Robotics toolbox
1. [Install Python 3.8](https://askubuntu.com/questions/1197683/how-do-i-install-python-3-8-in-lubuntu-18-04) (or any other compatible version)
2. Make a virtural environment
3. Install roboticstoolbox, ensuring pip resolves to the correct version of Python:
```bash
# in the venv
pip install roboticstoolbox-python

# if you want to install globally
# install all other pip packages this way if you want them globally
python3.8 -m pip install roboticstoolbox-python

```
4. May need to get other pre-req python packages first, or update pip. Packages you will probably need: 
   - wheel; for the error: 
        ```
        error: invalid command 'bdist_wheel'
        ```
   - setuptools; for the [warning](https://stackoverflow.com/questions/11425106/python-pip-install-fails-invalid-command-egg-info):
        ```
        egg_info for package roboticstoolbox-python produced metadata for project name unknown
        ``` 
     - If you get this warning you might need to remove the failed installation named UNKNOWN
        ```bash
        # check for a package named UNKNOWN
        pip list
        # remove it
        pip uninstall UNKNOWN

        # may also need to clean pip cache
        rm -r ~/.cache/pip

        # or remove the package manually from your venv site-packages directory
        cd {YOUR_VENV_DIRECTORY}/lib/python3.8/site-packages
        # find and remove the package folder, the name can be something like this
        rm -r UNKNOWN-0.0.0.dist-info
        ```
     - if the issue persists upgrade pip:
        ```
        pip install --upgrade pip
        ```
   - packages from errors that say: `no module named {PACAKGE NAME}`, such as numpy/Cython, etc

## Get armer
1. Install [Armer drivers](https://github.com/qcr/armer) and [Armer for UR5 (and UR5 drivers if not already)](https://github.com/qcr/armer_ur)
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
- **NOTE: replace the `DPYTHON_EXECUTABLE` arg with the executable in your venv directory if you wish to keep everything in the venv. Otherwise, packages needed for `catkin_make` will need to be installed globally.**
4. Source
5. Can try Swift simulation by running:
```bash
roslaunch armer armer.launch config:={PATH_TO_CONFIG_YAML_FILE}
```
6. For more instructions and tutorials visit [Armer documentations](https://open.qcr.ai/armer/index.html)

# Issues

## Controller conflict
moveit: scaled_pos_joint_traj_controller?
armer: joint_group_vel_controller?
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/117  
http://wiki.ros.org/controller_manager  
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/102  
https://answers.ros.org/question/200777/using-both-jointtrajectorycontroller-and-jointpositioncontroller-at-same-time-in-ros_control/  
