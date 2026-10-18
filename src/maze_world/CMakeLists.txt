cmake_minimum_required(VERSION 3.8)
project(maze_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Nothing to compile: install the launch file, maze model and world so that
# 'ros2 launch maze_world maze_world.launch.py' can find them

install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
