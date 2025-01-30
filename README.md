# RASPV: A Robotics Framework for Augmented Simulated Prosthetic Vision

**Paper:** https://ieeexplore.ieee.org/abstract/document/10411899 

**Authors:** Alejandro Perez-Yus, Maria Santos-Villafranca, Julia Tomas-Barba, Jesus Bermudez-Cameo, Lorenzo Montano-Olivan, Gonzalo Lopez-Nicolas, Josechu Guerrero

**Affiliation:** Universidad de Zaragoza, Spain

**Contact:** send an email to alperez@unizar

**Cite:** Please, if you are going to use this work, include this citation:
```
@article{perez2024raspv,
  title={RASPV: A Robotics Framework for Augmented Simulated Prosthetic Vision},
  author={Perez-Yus, Alejandro and Santos-Villafranca, Maria and Tomas-Barba, Julia and Bermudez-Cameo, Jesus and Montano-Olivan, Lorenzo and Lopez-Nicolas, Gonzalo and Guerrero, Jose J},
  journal={IEEE Access},
  year={2024},
  volume={12},
  number={},
  pages={15251-15267}
}
```

This package includes all the robotics framework to perform experimentation with Simulated Prosthetic Vision (SPV) according to our published paper. Particularly, it includes the creation of phosphene images and all the RGB-D processing and codification in phosphene patterns for several visualization proposals. It can work standalone with a standard RGB-D camera (e.g. Asus Xtion Pro Live with OpenNI2) or in simulation with our modified version of the ROS Turtlebot package for Gazebo (`blindbot` package). Here we include four packages:
- `raspv/` : The main package including the SPV and the pointcloud and image processing.
- `blindbot/` : Our humanized robot for our virtual environment implementation
- `navigation/` : The navigation package for the navigation experiments

Instructions are given below. If you see anything broken, you can post an issue or directly contact the authors. If you find something missing, it may be not ready for publication yet. Feel free to contact the authors if you need more information.https://meet.google.com/pnt-vock-tby

Tried on:
- Ubuntu 16.04 with ROS Kinetic
- Ubuntu 18.04 with ROS Melodic
- Ubuntu 20.04 with ROS Noetic

Note: If you have ROS Kinetic/Melodic/Noetic desktop-full installed with the workspace already set up, jump to point 3!

## 1. Install ROS
*Note: This explanation is for Noetic but it's the similar for Kinetic/Melodic and probably others*

Following the link:
http://wiki.ros.org/noetic/Installation/Ubuntu (change 'noetic' for your distro)

Step by step:
- Set up sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
- Set up keys
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
- Update
```
sudo apt update
```
- Install desktop-full version:
```
sudo apt install ros-noetic-desktop-full
```
- Environment setup:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Configure workspace:

Following the link: 
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace

Step by step:
- Create workspace in your home directory (here called `raspv_ws`)
```
$ mkdir -p ~/raspv_ws/src
$ cd ~/raspv_ws/
$ catkin_make
```
- Environment setup:
```
echo "source ~/raspv_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- Confirm workspace is set up correctly
```
echo $ROS_PACKAGE_PATH
```
you should see something like:
```
/home/user/raspv_ws/src:/opt/ros/kinetic/share
```

Alternatively, instead of `catkin_make`it can also be installed with `catkin build`:
https://catkin-tools.readthedocs.io/en/latest/installing.html
https://catkin-tools.readthedocs.io/en/latest/quick_start.html
```
cd ~/raspv_ws/
catkin init

```

## 3. Download packages:
These packages should be in placed in the `raspv_ws/src` folder (`cd ~/raspv_ws/src` if necessary). If you have your workspace named differently, substitute `raspv_ws` for your workspace name from here on.

The simulated prosthetic vision implementation for RGB-D cameras is in this package. To download:
```
git clone https://github.com/aperezyus/RASPV.git
```

## 4.1. Compile RASPV

To compile RASPV, go to the workspace folder and do `catkin_make`.
```
cd ~/raspv_ws
catkin_make
```
or if using `catkin build` write instead
```
cd ~/catkin_ws
catkin build
```

Package `raspv` should be compiled right out of the box with ROS desktop full.

In next subsections we specify another libraries that may be required depending on your desired use:

## 4.2. To install Blindbot

To install Blindbot, make sure you have installed the additional packages:
```
sudo apt-get install ros-noetic-joy
```
## 4.3. To install Navigation

In particular, you will need to install additional packages:
```
sudo apt-get install ros-noetic-pointcloud-to-laserscan
sudo apt-get install ros-noetic-map-server
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-dwa-local-planner

```

## 5. To run experiments:

### Virtual experiment

For example, in order to run the program in a Gazebo simulation:

1. Launch ROS:
```
roscore
```
2. Launch the **Gazebo simulation of blindbot** in a realistic house environment
```
roslaunch blindbot_gazebo sweet_house3.launch

```

By default, running the above command should launch Gazebo, and .

3. To **control the robot**, you can launch the keyboard_teleop in another terminal:
```
roslaunch blindbot_teleop keyboard_teleop_angles.launch
```
Instructions to move the robot and the head (well, the camera) are given in the same terminal.

Alternatively, if the user wants to use a **gamepad**, open new terminal and launch controller node (may require some previous configuration):
```
roslaunch blindbot_teleop joy_teleop.launch
```
4. Launch **RASPV**, including **phosphene/mode visualizations**
```
roslaunch raspv SPV.launch
```
Instructions are given on the terminal to manage visualization and change parameters of the prosthesis. By default, the configuration of the prosthesis is chosen from /config folder (that you can change, not needing to re-compile).

5. Launch **navigation package**.

In order to introduce **navigation goals** (e.g. doors, tables), you can launch `SPV_goals.launch` instead of `SPV.launch` to see how goals are added. It is currently programmed it with a few assistant modes/goals that work with sweet_home3 map.

Then, run the navigation package itself:

```
roslaunch navigation navigation_sweet3.launch
```

It should open RVIZ already configured to visualize the map, the costmaps, the path, the robot, and the live pointcloud.

By default, a goal is pre-established (the kitchen door). You can configure that in the `SPV_goals.launch` file.


6. Alternatively... **LAUNCH ALL TOGETHER!**

It may look chaotic in the terminal, but if you want to save time, you can just launch the included `RASPV_blindbot_navigation.launch` that is included in the root folder of the package.


### Real experiment

You can also make this code work with an RGB-D camera, such as the ASUS Xtion Pro Live. You would need to install first the OpenNI2 package:
```
sudo apt-get install ros-noetic-openni2-launch
```
And then launch the camera driver itself:
```
roslaunch openni2_launch openni2.launch depth_registration:=true
```
To make this execution mode work with RASPV, modify the SPV.launch so that USE_MODE_GAZEBO is set to false, or, alternatively:
```
roslaunch raspv SPV.launch USE_MODE_GAZEBO:=false
```
And then the program expects a camera. Notice that, both Gazebo and OpenNI2 drivers use the same type of messages, so the code should work out of the box.



