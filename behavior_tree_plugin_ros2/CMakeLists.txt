cmake_minimum_required(VERSION 3.8)
project(behavior_tree_plugin_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(ROS2_DEPENDENCIES
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  action_msgs
  actionlib_msgs
  rosidl_default_generators
  message_serialization
  ros2_babel_fish
)

find_package(ament_cmake REQUIRED)

foreach(dep ${ROS2_DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()

find_package(rclcpp REQUIRED)

ament_package()

#####################################
############## Build ################
#####################################

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${action_msgs_INCLUDE_DIRS}
  ${actionlib_msgs_INCLUDE_DIRS}
  ${message_serialization_INCLUDE_DIRS}
  ${ros2_babel_fish_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/call_any_service.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  action_msgs
  actionlib_msgs
  message_serialization
  ros2_babel_fish
)

#####################################
############# Install ###############
#####################################

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

#####################################
############# Testing ###############
#####################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(plan_joints_trajectory_test
    test/sth_unittest.cpp
  )
  target_link_libraries(plan_joints_trajectory_test ${PROJECT_NAME})
endif()

ament_package()
