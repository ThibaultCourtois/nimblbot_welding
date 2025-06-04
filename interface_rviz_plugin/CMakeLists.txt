cmake_minimum_required(VERSION 3.8)
project(interface_rviz_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Include Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_wrap_cpp(MOC_FILES
  include/interface_rviz_plugin/interface_rviz_plugin.hpp
)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Build the panel plugin
add_library(${PROJECT_NAME} SHARED
  src/interface_rviz_plugin.cpp
  ${MOC_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  rviz_common
  pluginlib
  tf2_geometry_msgs
  tf2
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  ${rviz_common_LIBRARIES}
)

# Export plugin description
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
