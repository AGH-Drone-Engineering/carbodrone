cmake_minimum_required(VERSION 3.8)
project(carbodrone_dualtech)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mission_node src/mission.cpp)
ament_target_dependencies(mission_node
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  std_srvs
)
message(STATUS "Eigen3_LIBRARIES: ${Eigen3_LIBRARIES}")
target_link_libraries(mission_node ${OpenCV_LIBS} Eigen3::Eigen)
target_include_directories(mission_node PRIVATE src)

install(TARGETS
  mission_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  worlds
  launch
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
