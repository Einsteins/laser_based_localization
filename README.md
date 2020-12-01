# laser_based_localization
This project introduces a localization algorithm mainly based on Lidars (Light detection and ranging) to solve the localization problem of a multi robots system.
Demo link (mire test demo):https://youtu.be/0XRXd4s8keA
# 1. Package overview
* `launch`: two test demo (mir test and gazebo test) and two launch file for scan_fusion and laser_localization. 
* `map`: two map data (.txt) for mir test and gazebo test
* `msg`: self defined message type 
* `src`: algorithm scripts
* `bagfile`: two ros bag file for mir test and gazebo test


# 2. Installation
### Preliminaries
The instructions below use the ROS distro `melodic` as an example.
If you haven't already installed ROS on your PC, you need to add the ROS apt
repository. 
For ROS installation http://wiki.ros.org/melodic/Installation/Ubuntu   

### Install
cd catkin_workspace/src

git clone 

cd ..

catkin_make

### Install all dependencies
The version of python for program is V 2.7 , some libraries are required for program

sudo apt update

sudo apt install python-pip

pip install --upgrade pip

python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose

pip install -U scikit-learn

# 3. Start  Node 
### laser_fusion
roslaunch laser_based_localization laser_fusion.launch

This Node subscribes two laser topics of the master robot , and publish the fused laser as topic /robots_scan_fusion.

The topic name and link of two laser topics can be defeined in launch file.

### laser_localization
roslaunch laser_based_localization laser_localization.launch

This Node subscribe /robots_scan_fusion and cal

# 4. Start  demo (existing bagfile)
### DON'T FORGET change the data path in launch file , otherwise it can't find the file
1. Gazebo test demo  

roslaunch laser_based_localization gazebo_test_demo.launch


2. Mir test demo

roslaunch laser_based_localization mir_test_demo.launch


