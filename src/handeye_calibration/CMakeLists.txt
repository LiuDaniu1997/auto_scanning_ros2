cmake_minimum_required(VERSION 3.8)
project(handeye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(deburring_robot_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(handeye_estimation_server SHARED src/handeye_estimation_server.cpp)
target_include_directories(handeye_estimation_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(handeye_estimation_server
  PRIVATE "HANDEYE_ESTIMATION_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(handeye_estimation_server
  "deburring_robot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "tf2_geometry_msgs")
rclcpp_components_register_node(handeye_estimation_server 
  PLUGIN "handeye_calibration::HandeyeEstimationServer" 
  EXECUTABLE handeye_estimation_server_node
)
target_link_libraries(handeye_estimation_server yaml-cpp)

add_library(handeye_estimation_client SHARED
  src/handeye_estimation_client.cpp)
target_include_directories(handeye_estimation_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(handeye_estimation_client
  PRIVATE "HANDEYE_ESTIMATION_CPP_BUILDING_DLL")
ament_target_dependencies(handeye_estimation_client
  "deburring_robot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "tf2"
  "tf2_geometry_msgs")
rclcpp_components_register_node(handeye_estimation_client 
  PLUGIN "handeye_calibration::HandeyeEstimationClient" 
  EXECUTABLE handeye_estimation_client_node)
target_link_libraries(handeye_estimation_client yaml-cpp)

add_executable(handeye_solver src/handeye_solver.cpp)
ament_target_dependencies(handeye_solver "rclcpp" "OpenCV" "tf2" "tf2_ros" "geometry_msgs" "tf2_geometry_msgs")
target_link_libraries(handeye_solver yaml-cpp)
install(TARGETS handeye_solver
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  handeye_estimation_server  
  handeye_estimation_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch data config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
