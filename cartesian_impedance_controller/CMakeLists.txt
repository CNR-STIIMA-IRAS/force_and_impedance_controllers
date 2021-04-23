cmake_minimum_required(VERSION 2.8.3)
project(cartesian_impedance_controller)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  rosdyn_core
  roscpp
  sensor_msgs
  subscription_notifier
  cnr_hardware_interface
  name_sorting
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface cnr_hardware_interface rosdyn_core roscpp sensor_msgs
  DEPENDS
)
  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cartesian_impedance_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

