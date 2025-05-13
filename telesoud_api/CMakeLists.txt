cmake_minimum_required(VERSION 3.8)
project(telesoud_api)

find_program(CCACHE ccache)

if(CCACHE)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE}")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rpclib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(gpio_controllers_iron REQUIRED)
find_package(std_msgs REQUIRED)
find_package(telesoud_msgs REQUIRED)

include_directories(${telesoud_msgs_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(${PROJECT_NAME}
  src/api.cpp
  src/api_node.cpp
  src/dataFromAndToTelesoudTranslator.cpp
  include/api_node.hpp
  include/communicationWithTelesoudStructures.hpp
  include/dataFromAndToTelesoudTranslator.hpp
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS} 
rclcpp telesoud_msgs control_msgs geometry_msgs trajectory_msgs tf2 tf2_ros gpio_controllers_iron controller_manager_msgs)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
)

target_link_libraries(${PROJECT_NAME} rpclib::rpc)

#############
## Install ##
#############

# Install executables
install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION 
    lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
