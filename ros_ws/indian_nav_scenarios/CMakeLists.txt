cmake_minimum_required(VERSION 3.0.2)
project(indian_nav_scenarios)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  gazebo_plugins
  robot_state_publisher
  joint_state_publisher
  rviz
  map_server
  amcl
  move_base
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    gazebo_ros_control
    gazebo_plugins
    robot_state_publisher
    joint_state_publisher
    rviz
    map_server
    amcl
    move_base
    nav_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  FILES_MATCHING PATTERN "*.world"
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install models
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
