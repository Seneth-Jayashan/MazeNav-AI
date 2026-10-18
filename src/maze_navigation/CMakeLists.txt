cmake_minimum_required(VERSION 3.8)
project(maze_navigation)

find_package(ament_cmake REQUIRED)

# Nothing to compile: install the Nav2/SLAM parameters, launch files, saved map and RViz view

install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

# Wheel + gyro odometry used by slam.launch.py while mapping
install(PROGRAMS scripts/imu_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
