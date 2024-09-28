cmake_minimum_required(VERSION 3.8)
project(carbodrone_mars)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

add_compile_definitions(YOLO_BALLS_640_PATH="${CMAKE_SOURCE_DIR}/src/yolo_balls_640.onnx")
add_compile_definitions(YOLO_BANNER_640_PATH="${CMAKE_SOURCE_DIR}/src/yolo_banner_640.onnx")
add_compile_definitions(MAP_UPLOADER_PROGRAM_PATH="${CMAKE_SOURCE_DIR}/src/map_uploader.py")
link_libraries(yolocpp)

add_executable(ball_detector src/ball_detector.cpp)
ament_target_dependencies(ball_detector rclcpp px4_ros_com sensor_msgs geometry_msgs cv_bridge px4_msgs tf2 tf2_ros image_geometry image_transport)
target_link_libraries(ball_detector ${OpenCV_LIBS})

add_executable(mission_node src/mission.cpp)
ament_target_dependencies(mission_node rclcpp px4_ros_com sensor_msgs geometry_msgs cv_bridge px4_msgs tf2 tf2_ros image_geometry image_transport std_srvs)
target_include_directories(mission_node PRIVATE src)

install(TARGETS
  ball_detector
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
