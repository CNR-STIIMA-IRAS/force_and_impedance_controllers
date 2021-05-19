cmake_minimum_required(VERSION 3.1)

project(joint_impedance_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  cnr_controller_interface
  controller_manager
  hardware_interface
  rosdyn_core
  roscpp
  sensor_msgs
  subscription_notifier
  cnr_hardware_interface
  name_sorting
  eigen_matrix_utils
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES joint_impedance_controller
  CATKIN_DEPENDS cnr_controller_interface controller_interface controller_manager hardware_interface cnr_hardware_interface rosdyn_core roscpp sensor_msgs subscription_notifier name_sorting eigen_matrix_utils
  DEPENDS 
)



include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/joint_impedance_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



