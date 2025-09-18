# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.0.2)
project(argus_tasks)

# Find the catkin packages this package depends on, including python support
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  move_base_msgs
)

# This is a crucial function that sets up the package for the build system
# The CATKIN_DEPENDS are important for other packages that might depend on this one
catkin_package(
  CATKIN_DEPENDS rospy std_msgs actionlib move_base_msgs
)

# This is the key part for Python scripts.
# It tells the build system to install everything in the 'scripts' folder
# so that they can be found and executed by rosrun.
catkin_install_python(PROGRAMS
  scripts/patrol_node.py
  scripts/monitoring_node.py
  scripts/alert_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)