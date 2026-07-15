cmake_minimum_required(VERSION 3.8)
project(linker_a7_ros2_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/a7_lite_can.cpp
  src/a7_lite_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(a7_lite_safety_node src/a7_lite_safety_node.cpp)
target_link_libraries(a7_lite_safety_node ${PROJECT_NAME})
ament_target_dependencies(a7_lite_safety_node rclcpp std_srvs)

pluginlib_export_plugin_description_file(
  hardware_interface linker_a7_ros2_control.xml
)

install(
  TARGETS ${PROJECT_NAME} a7_lite_safety_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES linker_a7_ros2_control.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_a7_lite_protocol test/test_a7_lite_protocol.cpp)
  if(TARGET test_a7_lite_protocol)
    target_link_libraries(test_a7_lite_protocol ${PROJECT_NAME})
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
