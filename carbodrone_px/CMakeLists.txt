cmake_minimum_required(VERSION 3.8)
project(carbodrone_px)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters)

add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher rclcpp px4_msgs geometry_msgs nav_msgs px4_ros_com tf2 tf2_ros)

add_executable(ground_pub src/ground_pub.cpp)
ament_target_dependencies(ground_pub rclcpp px4_msgs geometry_msgs px4_ros_com tf2 tf2_ros message_filters)

add_executable(home_pub src/home_pub.cpp)
ament_target_dependencies(home_pub rclcpp px4_msgs geometry_msgs px4_ros_com tf2 tf2_ros)

add_executable(imu src/imu.cpp)
ament_target_dependencies(imu rclcpp sensor_msgs px4_msgs px4_ros_com message_filters)

install(TARGETS
  odom_publisher
  ground_pub
  home_pub
  imu
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  models
  config
  rviz
  DESTINATION share/${PROJECT_NAME})

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
