# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.0.2)
project(argus_gazebo)

# Find the catkin packages this package depends on
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
)

# This is a crucial function that sets up the package for the build system
catkin_package()

# Install launch and worlds directories so roslaunch can find them
install(DIRECTORY launch/ worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# If you store saved maps here, you can install the maps directory as well
install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)