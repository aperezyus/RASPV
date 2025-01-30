################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(blindbot_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  ## Added for kobuki_gazebo_plugins
  gazebo_plugins
  # kobuki_msgs
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_kobuki
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros 
    ## Added for kobuki_gazebo_plugins
    gazebo_plugins
    # kobuki_msgs
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

#add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
#add_dependencies(turtlebot3_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(turtlebot3_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(groundtruth_node src/groundtruth_node.cpp)
target_link_libraries(groundtruth_node  ${catkin_LIBRARIES})

# Install kobuki_gazebo_plugins
add_library(gazebo_ros_kobuki src/gazebo_ros_kobuki.cpp
                              src/gazebo_ros_kobuki_updates.cpp
                              src/gazebo_ros_kobuki_loads.cpp)
add_dependencies(gazebo_ros_kobuki ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_kobuki
                      ${catkin_LIBRARIES}
                      ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_ros_kobuki
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


# Install kobuki_gazebo_plugins
add_library(gazebo_ros_kobuki_bl src/gazebo_ros_kobuki.cpp
                              src/gazebo_ros_kobuki_updates.cpp
                              src/gazebo_ros_kobuki_loads_bl.cpp)
add_dependencies(gazebo_ros_kobuki_bl ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_kobuki_bl
                      ${catkin_LIBRARIES}
                      ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_ros_kobuki_bl
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(force_joint SHARED src/force_joint.cpp)
target_link_libraries(force_joint ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

################################################################################
# Install
################################################################################
#install(TARGETS turtlebot3_drive
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
