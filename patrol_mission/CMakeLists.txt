cmake_minimum_required(VERSION 3.8)
project(patrol_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

# Patrol manager as a composable component (+ standalone executable)
add_library(patrol_manager_component SHARED src/patrol_manager.cpp)
target_include_directories(patrol_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(patrol_manager_component ${dependencies})

rclcpp_components_register_node(patrol_manager_component
  PLUGIN "patrol_mission::PatrolManager"
  EXECUTABLE patrol_manager_node
)

install(TARGETS patrol_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_battery_model test/test_battery_model.cpp)
  target_include_directories(test_battery_model PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(patrol_manager_component)
ament_export_dependencies(${dependencies})

ament_package()
