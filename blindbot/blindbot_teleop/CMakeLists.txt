cmake_minimum_required(VERSION 2.8.3)
project(blindbot_teleop)

# ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib nodelet geometry_msgs nav_msgs joy)

# pkg-config support
# find_package(PkgConfig)
# pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
# add_definitions(-DHAVE_NEW_YAMLCPP)
# endif()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pluginlib nodelet geometry_msgs nav_msgs joy
  LIBRARIES teleop_twist_joy yocs_velocity_smoother_nodelet #yocs_cmd_vel_mux_nodelet
  DEPENDS #yaml-cpp
)

# include_directories(include ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

# Add teleop_twist_joy
add_library(teleop_twist_joy src/teleop_twist_joy.cpp)
target_link_libraries(teleop_twist_joy ${catkin_LIBRARIES})
add_executable(teleop_twist_joy_node src/teleop_node.cpp)
target_link_libraries(teleop_twist_joy_node teleop_twist_joy ${catkin_LIBRARIES})
set_target_properties(teleop_twist_joy_node PROPERTIES OUTPUT_NAME teleop_node PREFIX "")

# Nodelet library
add_library(yocs_velocity_smoother_nodelet src/velocity_smoother_nodelet.cpp)
target_link_libraries(yocs_velocity_smoother_nodelet ${catkin_LIBRARIES})
# set_target_properties(yocs_velocity_smoother_nodelet PROPERTIES OUTPUT_NAME yocs_velocity_smoother_nodelet PREFIX "")

# add_dependencies(yocs_velocity_smoother_nodelet ${PROJECT_NAME}_gencfg)

# # Nodelet library
# add_library(yocs_cmd_vel_mux_nodelet src/cmd_vel_mux_nodelet.cpp src/cmd_vel_subscribers.cpp)
# add_dependencies(yocs_cmd_vel_mux_nodelet geometry_msgs_gencpp)
# # add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)
# target_link_libraries(yocs_cmd_vel_mux_nodelet ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})
#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
execute_process(COMMAND chmod a+x scripts/blindbot_teleop_key)
install(PROGRAMS
  scripts/blindbot_teleop_key
  # PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

execute_process(COMMAND chmod a+x scripts/blindbot_teleop_angles_key)
install(PROGRAMS
  scripts/blindbot_teleop_angles_key
  # PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# message(${CATKIN_PACKAGE_LIB_DESTINATION})
# message(${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# message(${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS yocs_velocity_smoother_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)
install(DIRECTORY include/yocs_velocity_smoother/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(FILES plugins/nodelets.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
