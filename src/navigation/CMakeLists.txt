# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.0.2)
project(argus_navigation)

# Find the catkin packages this package depends on
# We list all the dependencies from your package.xml here
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  gmapping
  map_server
  move_base
  amcl
)

# This is a crucial function that sets up the package for the build system
catkin_package()

# Install launch and parameter directories
install(DIRECTORY launch/ params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)