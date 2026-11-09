cmake_minimum_required(VERSION 3.8)
project(cat_patrol_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(patrol_node
  src/patrol_node.cpp
  src/patterns/classic_pattern.cpp
  src/patterns/till_obstacle_back_pattern.cpp
)
target_include_directories(patrol_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(patrol_node
  rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge vision_msgs
)
target_link_libraries(patrol_node ${OpenCV_LIBS})

install(TARGETS patrol_node DESTINATION lib/${PROJECT_NAME})

add_executable(nav_goal_client_node src/nav_goal_client_node.cpp)
ament_target_dependencies(nav_goal_client_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs
)
install(TARGETS nav_goal_client_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch systemd DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/run_cat_patrol.sh
  scripts/play_bark.sh
  scripts/connect_usb_speaker.sh
  scripts/connect_bt_speaker.sh
  scripts/enable_bt_a2dp.sh
  scripts/mail_node
  scripts/voice_node
  scripts/check_active_config.sh
  scripts/patrol
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
