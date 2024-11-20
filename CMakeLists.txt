cmake_minimum_required(VERSION 3.5)
project(cartesian_impedance_controller)

find_package(Boost 1.49 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(Eigen3 REQUIRED eigen3)

pkg_check_modules(RBDyn REQUIRED RBDyn)

pkg_check_modules(SpaceVecAlg REQUIRED SpaceVecAlg)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerConfig.msg"
  "msg/ControllerState.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(${PROJECT_NAME}_lib SHARED src/cartesian_impedance_controller.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  Boost
  Eigen3
  SpaceVecAlg
  tinyxml2
  rosidl_default_runtime
  rosidl_typesupport_cpp)

add_library(${PROJECT_NAME}_ros SHARED src/cartesian_impedance_controller_ros.cpp)
ament_target_dependencies(${PROJECT_NAME}_ros
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  realtime_tools
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  trajectory_msgs
  Boost
  Eigen3
  SpaceVecAlg
  tinyxml2
  rosidl_default_runtime
  rosidl_typesupport_cpp)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SpaceVecAlg_INCLUDE_DIRS}
  ${RBDyn_INCLUDE_DIRS}
  ${tinyxml2_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SpaceVecAlg_INCLUDE_DIRS}
  ${RBDyn_INCLUDE_DIRS}
  ${tinyxml2_INCLUDE_DIRS}
)
link_directories("/home/robohub/depend.away/RBDyn/build/src/RBDyn" "/home/robohub/depend.away/RBDyn/build/src/parsers")

target_link_libraries(${PROJECT_NAME}_lib 
  ${Eigen3_LIBRARIES}
  RBDyn
  RBDynParsers
  ${cpp_typesupport_target}
  tinyxml2
  )

target_link_libraries(${PROJECT_NAME}_ros 
  ${PROJECT_NAME}_lib 
  ${Eigen3_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${tinyxml2_LIBRARIES} 
  RBDyn
  RBDynParsers
  yaml-cpp
  ${cpp_typesupport_target}
  tinyxml2
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_ros
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Install plugin XML
install(FILES controller_plugins.xml
  DESTINATION share/plugin)

# Export plugin information for pluginlib
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

ament_export_include_directories(include ${TINYXML2_LIBRARIES})
ament_export_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_ros ${TINYXML2_LIBRARIES})
ament_export_dependencies(rosidl_default_runtime
rclcpp
std_msgs
sensor_msgs
geometry_msgs
control_msgs
controller_interface
controller_manager
hardware_interface
pluginlib
realtime_tools
tf2
tf2_geometry_msgs
tf2_ros
trajectory_msgs
Boost
Eigen3
SpaceVecAlg)

ament_package()
