# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.0.2)
project(argus_description)

# Find the catkin packages this package depends on
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  xacro
)

# This is a crucial function that sets up the package for the build system
catkin_package()

# Install launch and urdf directories into the package's share space
# This makes them findable by roslaunch and other tools
install(DIRECTORY launch/ urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)