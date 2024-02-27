cmake_minimum_required(VERSION 3.8)
project(lane_keep_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pacmod3_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lanelet2_extension REQUIRED)

set(msg_files
  "msg/Derivatives.msg"
  "msg/Polynomial.msg"
  "msg/Scenario.msg"
  "msg/PointList.msg"
  "msg/Trajectory.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces rosidl_default_generators
 )

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  autoware_auto_planning_msgs
  autoware_auto_vehicle_msgs
  autoware_adapi_v1_msgs
  visualization_msgs
  pacmod3_msgs
  tf2_ros
  tf2_geometry_msgs
  lanelet2_extension
)

set(lanelet_map_handler_sources
  src/linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.cpp
  src/linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.cpp
  src/utilities/rosUtilities.cpp
)

set(trajectory_planner_sources
  src/linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.cpp
  src/linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.cpp
  src/linearDriverModelDriverModel/emg_linearDriverModel_driverModel.cpp
  src/linearDriverModelPlanner/emg_linearDriverModel_segmentPlanner.cpp
  src/utilities/rosUtilities.cpp
  src/emg_linearDriverModel_controlLogic.cpp
  src/emg_linearDriverModel.cpp
)

include_directories(
  include
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(lanelet_map_handler src/laneletHandler/laneletMapHandler.cpp ${lanelet_map_handler_sources})
ament_target_dependencies(lanelet_map_handler ${dependencies})
target_link_libraries(lanelet_map_handler "${cpp_typesupport_target}")

add_executable(trajectory_planner src/trajectoryPlanner.cpp ${trajectory_planner_sources})
ament_target_dependencies(trajectory_planner ${dependencies})
target_link_libraries(trajectory_planner "${cpp_typesupport_target}")

add_executable(waypoint_converter src/utilities/waypoint_converter.cpp)
ament_target_dependencies(waypoint_converter rclcpp geometry_msgs autoware_auto_planning_msgs)
target_link_libraries(waypoint_converter "${cpp_typesupport_target}")

# add_executable(mpc_steering_path src/utilities/mpc_steering_path.cpp ${trajectory_planner_sources})
# ament_target_dependencies(mpc_steering_path ${dependencies})


install(TARGETS
  lanelet_map_handler
  trajectory_planner
  waypoint_converter
  # mpc_steering_path
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  config
  laneletMaps
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()