# RASPV: A Robotics Framework for Augmented Simulated Prosthetic Vision

**Paper:** https://ieeexplore.ieee.org/abstract/document/10411899 

**Authors:** Alejandro Perez-Yus, Maria Santos-Villafranca, Julia Tomas-Barba, Jesus Bermudez-Cameo, Lorenzo Montano-Olivan, Gonzalo Lopez-Nicolas, Josechu Guerrero

**Affiliation:** Universidad de Zaragoza, Spain

**Contact:** send an email to alperez@unizar

This package includes all the robotics framework to perform experimentation with Simulated Prosthetic Vision (SPV) according to our published paper. Particularly, it includes the creation of phosphene images and all the RGB-D processing and codification in phosphene patterns for several visualization proposals. It can work standalone with a standard RGB-D camera (e.g. Asus Xtion Pro Live with OpenNI2) or in simulation with our modified version of the ROS Turtlebot package for Gazebo (`blindbot` package). Also, som related packages such as the modified ROS `navigation_RASPV` package are included as well.

Instructions are given below. If you find something that is not included, it may be in development and not ready for publication yet. If you see anything broken, you can post an issue or directly contact the authors.

Tried on:
- Ubuntu 16.04 with ROS Kinetic
- Ubuntu 18.04 with ROS Melodic
- Ubuntu 20.04 with ROS Noetic

Note: If you have ROS Kinetic/Melodic/Noetic desktop-full installed with the workspace already set up, jump to point 3!

## 1. Install ROS
*Note: This explanation is for Kinetic but it's the same for Melodic and probably others*

Following the link:
http://wiki.ros.org/kinetic/Installation/Ubuntu

Step by step:
- Set up sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
- Set up keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
- Update
```
sudo apt-get update
```
- Install desktop-full version:
```
sudo apt-get install ros-kinetic-desktop-full
```
- Environment setup:
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Configure workspace:

Following the link: 
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace

Step by step:
- Create workspace in your home directory (here called `catkin_ws`)
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
- Environment setup:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- Confirm workspace is set up correctly
```
echo $ROS_PACKAGE_PATH
```
you should see something like:
```
/home/user/catkin_ws/src:/opt/ros/kinetic/share
```

Alternatively, instead of `catkin_make`it can also be installed with `catkin build`:
https://catkin-tools.readthedocs.io/en/latest/installing.html
https://catkin-tools.readthedocs.io/en/latest/quick_start.html
```
cd ~/catkin_ws/
catkin init

```

## 3. Download packages:
These packages should be in placed in the `catkin_ws/src` folder (`cd ~/catkin_ws/src` if necessary). If you have your workspace named differently, substitute `catkin_ws` for your workspace name from here on.

The simulated prosthetic vision implementation for RGB-D cameras is in this package. To download:
```
git clone https://github.com/aperezyus/RASPV.git
```

## 4. Compile package

To compile, go to the workspace folder and do `catkin_make`.
```
cd ~/catkin_ws
catkin_make
```
or if using `catkin build` write instead
```
cd ~/catkin_ws
catkin build
```

Package `raspv` should be compiled right out of the box with ROS desktop full.



## 7. (OPTIONAL) Install Navigation

In particular, you will need to install additional packages:
```
sudo apt-get install ros-melodic-pointcloud-to-laserscan
sudo apt-get install ros-melodic-map-server
sudo apt-get install ros-melodic-move-base
sudo apt-get install ros-melodic-dwa-local-planner

```


## The rest? (in construction)


To make our turtlebot packages work, first the following packages should be installed:
```
sudo apt-get install ros-kinetic-gazebo-*
sudo apt-get install ros-kinetic-turtlebot-*
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-position-controllers
```

## 5. To run:

### Gazebo simulation

For example, in order to run the program in a Gazebo simulation:

1. Launch ROS:
```
roscore
```
2. Launch Gazebo, with our modified turtlebot and in a custom map:
```
roslaunch turtlebot_gazebo daniel.launch
```
3. Run our simulator!
```
rosrun rgbd_via SPV_Gazebo
```

Instructions are given on the terminal to control the robot with the keyboard and manage visualization.

Additionally, if the user wants to use a gamepad, open new terminal and launch controller node:
```
roslaunch teleop_twist_joy teleop_smooth.launch
```

### RGB-D camera (Asus Xtion Pro Live)

In this case, the second terminal should launch the OpenNI2 package instead:
```
roslaunch openni2_launch openni2.launch
```
The program to run is the following:
```
rosrun rgbd_via SPV
```

### With SLAM -- Not working

You can use a SLAM node working in parallel to be able to keep the system localized and enhance some visualizations (e.g. Checkerboard)

We have tested this with a modified version of ORBSLAM2, which you can download and install following the instructions here (link)

You will need a fourth terminal launching ORBSLAM2 node:
```
TODO
```

and run the following rgbd_via program:
```
rosrun rgbd_via SPV_SLAM
```
Notice that it should run both with simulation and with an RGB-D camera attached.

### With simulated SLAM -- Not working

SPV_Gazebo benefits from having the pose perfectly known given a Gazebo plugin (no need to extract floor, main directions, odometry). If you want to run this package more realistically as if you had a SLAM without having a SLAM installed, you can use the Gazebo plugin that publishes the pose of the camera to simulate just the odometry but compute everything else:
```
rosrun rgbd_via SPV_SLAM_Gazebo
```

### With stair detection and modeling -- Not working

We can use the RGB-D camera to perform detection of Staircases and show them in prosthetic vision with this package, using the Stair detection and modeling implementation from Perez-Yus et al. [cite].

You can test the stair detection with an RGB-D camera or in Gazebo running:
```
rosrun rgbd_via stairs
```
and with Simulated Prosthetic Vision with:
```
rosrun rgbd_via SPV_stairs
```

You can also test it in Gazebo, with simulated odometry, running the following program:
```
rosrun rgbd_via SPV_SLAM_Gazebo_stairs
```

### Navigation -- Not working

We have also implemented a Navigation package to allow us to display in prosthetic vision the path the user have to follow to reach the desired point given a known map.

Before, it is necessary to switch to Navigation branch and install the following ROS package:
```
sudo apt-get install pointcloud-to-laserscan
```
and our following implementation of the navigation node:
```
Lorenzo's modified navigation package
```

To run... (TODO)


