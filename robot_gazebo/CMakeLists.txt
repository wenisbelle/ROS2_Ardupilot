cmake_minimum_required(VERSION 3.8)
project(robot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Gazebo clock -> ROS /clock at a reduced rate (see src/clock_relay.cpp)
add_executable(clock_relay src/clock_relay.cpp)
target_link_libraries(clock_relay gz-transport::core gz-msgs::core)
ament_target_dependencies(clock_relay rclcpp rosgraph_msgs)
install(TARGETS clock_relay DESTINATION lib/${PROJECT_NAME})

# python helpers used by the launch files (robot_gazebo.world_info)
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY
    launch
    worlds
    models
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/set_wind
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
