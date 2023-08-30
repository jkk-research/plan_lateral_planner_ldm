cmake_minimum_required(VERSION 3.0.2)
project(lane_keep_system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  lanelet2_extension
  tf2_ros
  tf2_geometry_msgs
)

add_message_files(DIRECTORY msg
  FILES
    PolynomialCoeffs.msg)

add_service_files(DIRECTORY srv
  FILES
    GetLaneletScenario.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

set(trajectory_planner_dependencies
  src/linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.cpp
  src/linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.cpp
  src/linearDriverModelDriverModel/emg_linearDriverModel_driverModel.cpp
  src/linearDriverModelPlanner/emg_linearDriverModel_segmentPlanner.cpp
  src/emg_linearDriverModel_controlLogic.cpp
  src/emg_linearDriverModel.cpp
)

add_executable(lanelet_map_handler src/laneletHandler/laneletMapHandler.cpp src/linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.cpp src/linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.cpp)
target_link_libraries(lanelet_map_handler ${catkin_LIBRARIES})

add_executable(trajectory_planner src/trajectoryPlanner.cpp ${trajectory_planner_dependencies} )
target_link_libraries(trajectory_planner ${catkin_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(TARGETS lanelet_map_handler trajectory_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
