![Alt text](https://github.com/JoMiCos/ROS-ENKI_Rat_Behavioural_Sim/blob/master/SimInAction.png)


# ROS/ENKI Rat Behavioural Simulation

This repository provides a 2D behavioural simulator, constructed by combining a ROS-ENKI simulator (retrieved from https:://github.com/Lilypads/ROS-ENKI_robot_simulation) and a limbic-system computational model (retrieved from https://github.com/berndporr/reversal-5ht). It currently runs a reversal learning experiment but aims to be adaptable.

## Content
* __catkin_ws:__ line following robot simulator using Enki library catkin workspace directory
  * __rat_reversal.launch:__ launch file to run nodes involved with reversal learning experiment
  * __src:__ directory containing different nodes
  * __rat_move_pkg:__ visual signals read and motor signals produced and published to ROS topic 
  * __enki_ros_pck:__ enki environment and robot simulation
   * __Brain:__ limbic-system simulation files from __reversal-5ht__
     

## Prerequisites

### Install ROS Melodic

 * Note that each platform might have a slightly different installation method.
 See http://wiki.ros.org/melodic/Installation for supported platforms and their installation method.

The following method is specific for Ubuntu18.04(Bionic) platform.

1. Add GPG key of ROS to apt

        sudo wget -O - https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -

2. Add apt repository for ROS

        sudo apt-add-repository "deb http://packages.ros.org/ros/ubuntu bionic main"

3. Update repositories

        sudo apt-get update

4. Install the ROS Desktop

        sudo apt install ros-melodic-desktop-full

5. Set your environment variables

        echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
        source ~/.bashrc

6. Initialize ROS

        sudo apt install python-rosdep
        sudo rosdep init
        rosdep update
        

### Install Enki Library

Clone the library:
```
git clone https://github.com/berndporr/enki
```
Change directory to the library directory:
```
cd enki/enki
```
Make and install library:
```
qmake
make
sudo make install
```

## Build the repository workspace

Change directory to the workspace directory:
```
cd ROS-ENKI_robot_simulation/catkin_ws
```
Build the ROS package:
```
catkin_make
```

## Run the simulation

Change directory to the workspace directory:
```
cd ROS-ENKI_robot_simulation/catkin_ws
```

### Method1: Roslaunch (recommended) 

To launch rat_reversal nodes from catkin_ws:
```
source devel/setup.bash
roslaunch src/rat_reversal.launch
```

### Method2: Manually run roscore and rosrun each node on seperate terminals

Run roscore on one terminal:
```
roscore
```

Run the enki simulation environment on the second terminal:
```
source devel/setup.bash
rosrun enki_ros_pck robot
```

Run the rat_move node on the third terminal:
```
source devel/setup.bash
rosrun rat_move_pkg rat_move
```


### To access the log file of output data

Make hidden folders visible within your Home directory (can be done by pressing ctrl+h in Ubuntu)
Open the __.ros__ folder which should become visible.
log data can be found in limbic-log.dat


