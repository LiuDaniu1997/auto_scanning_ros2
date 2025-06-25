cmake_minimum_required(VERSION 3.8)
project(deburring_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(gz-transport14 REQUIRED)

install(
  DIRECTORY worlds models launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(set_model_pose_node src/set_model_pose_node.cpp)
ament_target_dependencies(set_model_pose_node rclcpp tf2)
target_link_libraries(set_model_pose_node gz-msgs11 gz-transport14)
install(TARGETS set_model_pose_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
