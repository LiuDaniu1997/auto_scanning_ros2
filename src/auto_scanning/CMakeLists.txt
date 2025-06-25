cmake_minimum_required(VERSION 3.8)
project(auto_scanning)

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
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation registration)
find_package(pcl_conversions REQUIRED)

add_library(auto_scanning_server SHARED src/auto_scanning_server.cpp)
target_include_directories(auto_scanning_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(auto_scanning_server
  PRIVATE "auto_scanning_server_CPP_BUILDING_DLL")
ament_target_dependencies(auto_scanning_server
  "deburring_robot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "pcl_conversions")
rclcpp_components_register_node(auto_scanning_server 
  PLUGIN "auto_scanning::AutoScanningServer" 
  EXECUTABLE auto_scanning_server_node
)
target_link_libraries(auto_scanning_server yaml-cpp ${PCL_LIBRARIES})

add_library(auto_scanning_client SHARED
  src/auto_scanning_client.cpp)
target_include_directories(auto_scanning_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(auto_scanning_client
  PRIVATE "AUTO_SCANNING_CPP_BUILDING_DLL")
ament_target_dependencies(auto_scanning_client
  "deburring_robot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "tf2"
  "tf2_geometry_msgs")
rclcpp_components_register_node(auto_scanning_client 
  PLUGIN "auto_scanning::AutoScanningClient" 
  EXECUTABLE auto_scanning_client_node)
target_link_libraries(auto_scanning_client yaml-cpp)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(pointcloud_merger_node src/pointcloud_merger_node.cpp)
ament_target_dependencies(pointcloud_merger_node 
  "rclcpp" 
  "pcl_conversions")
target_link_libraries(pointcloud_merger_node ${PCL_LIBRARIES} yaml-cpp)
install(TARGETS pointcloud_merger_node DESTINATION lib/${PROJECT_NAME})

add_executable(align_merged_clouds_node src/align_merged_clouds_node.cpp)
ament_target_dependencies(align_merged_clouds_node 
  "rclcpp" 
  "pcl_conversions")
target_link_libraries(align_merged_clouds_node ${PCL_LIBRARIES} yaml-cpp)
install(TARGETS align_merged_clouds_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  auto_scanning_server  
  auto_scanning_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

