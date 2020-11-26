# laser_based_localization
This project introduces a novel localization algorithm mainly based on Lidars (Light detection and ranging) to solve the localization problem of a multi robots system.
# 1. Package overview
* `launch`: two test demo (mir test and gazebo test) and two launch file for scan_fusion and laser_localization. 
* `map`: two map data (.txt) for mir test and gazebo test
* `msg`: self defined message type 
* `src`: algorithm scripts
* `bagfile`: two ros bag file for mir test and gazebo test


# 2. Installation
The instructions below use the ROS distro `kinetic` as an example.

### Preliminaries
If you haven't already installed ROS on your PC, you need to add the ROS apt
repository. This step is necessary for either binary or source install.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
```

### Binary install
For a binary install, it suffices to run this command:
```bash
sudo apt install ros-kinetic-mir-robot
sudo apt install ros-kinetic-navigation
```
See the tables at the end of this README for a list of ROS distros for which
binary packages are available.

### Source install
For a source install, run the commands below instead of the command from the
"binary install" section.
```bash
cd ~/catkin_ws/src/
```

### Clone mir_robot into the catkin workspace
```bash
git clone https://github.com/Jryl/MIR200_Sim_Demo.git
```
### Install all dependencies
sudo apt update
sudo apt install python-pip
pip install --upgrade pip
python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
pip install -U scikit-learn

### Build all packages in the catkin workspace
```bash
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo
```


# 3. Start Gazebo demo (existing map)
```bash
roslaunch mir_navigation mir_start.launch
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base.

# 4. Gazebo demo (mapping)
```bash
roslaunch mir_navigation mir_mapping.launch
```

# 5. Teleoperate the robot with keyboard (optional)
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
