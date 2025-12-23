cmake_minimum_required(VERSION 3.8)
project(robot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED) # السطر ده مهم جداًfind_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(robot_motion_node src/robot_motion_node.cpp)
add_executable(service_node src/service.cpp)

target_link_libraries(service_node PRIVATE)
ament_target_dependencies(service_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

target_include_directories(robot_motion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(robot_motion_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(service_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robot_motion_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "rclcpp_action"
)

ament_target_dependencies(
  service_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "rclcpp_action"
)

install(TARGETS robot_motion_node 
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  service_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
