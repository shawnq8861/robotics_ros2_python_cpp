cmake_minimum_required(VERSION 3.5)
project(qbot_nodes_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CameraCommand.srv"
 )

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(uvc_camera src/uvc_camera.cpp)
ament_target_dependencies(uvc_camera 
	rclcpp 
	std_msgs
	sensor_msgs
	OpenCV)
target_include_directories(uvc_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
add_executable(realtime_tests src/realtime_tests.cpp)
ament_target_dependencies(realtime_tests 
	rclcpp 
	std_msgs
	sensor_msgs)
target_include_directories(realtime_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(base_controller src/base_controller.cpp)
ament_target_dependencies(base_controller 
  rclcpp
  geometry_msgs)
target_include_directories(base_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS talker
  DESTINATION lib/${PROJECT_NAME}
)
  
install(TARGETS listener
  DESTINATION lib/${PROJECT_NAME}
)
  
install(TARGETS uvc_camera
  DESTINATION lib/${PROJECT_NAME}
)
  
install(TARGETS realtime_tests
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS base_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
