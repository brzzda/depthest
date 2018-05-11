cmake_minimum_required(VERSION 2.8.3)
project(depthest)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,/usr/local/lib")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -L/usr/local/lib -lcnpy -lz -o -std=c++11")
#g++ test.cpp -L/usr/local/lib -lcnpy -lz -o test
#set(Sophus_DIR /home/peetaa/ws2/Sophus/build)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_msgs
  tf2_eigen
  geometry_msgs
  message_filters
  visualization_msgs
  message_generation
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ObjectsStamped.msg
   FlowArrayStamped.msg
   FlowArrayStampedAged.msg
   Flow.msg
   Point2D.msg
   Point2DAged.msg
   PointsStamped.msg
   PoseAndPointsStamped.msg
#   Points3DStamped.msg
#   Point3D.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   visualization_msgs
   geometry_msgs
   std_msgs  # Or other packages containing msgs
   tf2_msgs
#   message_filters
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES depthest
#  CATKIN_DEPENDS roscpp rospy
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src/DepthEstimation
  src/Remapping
  src/Evaluation
  src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/depthest.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/depthest_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_depthest.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(depthest
        src/DepthEstimation/NodeHandles/DepthEstimationNode.h
        src/DepthEstimation/DepthEstimators/DepthEstimator.h
        src/DepthEstimation/DepthEstimatorFactory.h
        src/DepthEstimation/DepthEstimatorFactory.cpp
        src/DepthEstimation/Depthes.cpp
        src/DepthEstimation/NodeHandles/DepthEstimationNode.cpp
        src/Camera.cpp
        src/Camera.h
        src/DepthEstimation/DepthEstimators/FilterDepthEstimator.cpp
        src/DepthEstimation/DepthEstimators/FilterDepthEstimator.h
        src/HelperFunctions.h
        src/Evaluation/ErrorCalc.cpp
        src/Evaluation/ErrorCalc.h
        src/EstimatorMaps/Estimators/MedianEstimator.cpp
        src/EstimatorMaps/Estimators/MedianEstimator.h
        src/EstimatorMaps/Estimators/KalmanFilterEstimator.cpp
        src/EstimatorMaps/Estimators/KalmanFilterEstimator.h
        src/EstimatorMaps/Estimators/PositionEstimator.h
        src/EstimatorMaps/Estimators/MedianSWEstimator.cpp
        src/EstimatorMaps/Estimators/MedianSWEstimator.h
        src/EstimatorMaps/EstimatorPointMapFactory.cpp
        src/EstimatorMaps/EstimatorPointMapFactory.h
        src/EstimatorMaps/Estimators/DummyEstimator.cpp
        src/EstimatorMaps/Estimators/DummyEstimator.h
        src/EstimatorMaps/EstimatorPointMap.h
        src/EstimatorMaps/KalmanFilterEstimatorPointMap.cpp
        src/EstimatorMaps/KalmanFilterEstimatorPointMap.h
        src/EstimatorMaps/MedianEstimatorPointMap.cpp
        src/EstimatorMaps/MedianEstimatorPointMap.h
        src/EstimatorMaps/MedianSWEstimatorPointMap.cpp
        src/EstimatorMaps/MedianSWEstimatorPointMap.h
        src/EstimatorMaps/DummyEstimatorPointMap.cpp
        src/EstimatorMaps/DummyEstimatorPointMap.h
        src/Params.cpp
        src/Params.h
        src/PositionFilterMaps/PoseFilters/PositionFilter.h
        src/PositionFilterMaps/PoseFilters/PoseFilter.cpp
        src/PositionFilterMaps/PoseFilters/PoseFilter.h
        src/PositionFilterMaps/PoseFilters/PoseChangeFilter.cpp
        src/PositionFilterMaps/PoseFilters/PoseChangeFilter.h
        src/PositionFilterMaps/PoseFilters/SWPoseFilter.cpp
        src/PositionFilterMaps/PoseFilters/SWPoseFilter.h
        src/PositionFilterMaps/PoseFilters/SWPoseChangeFilter.cpp
        src/PositionFilterMaps/PoseFilters/SWPoseChangeFilter.h
        src/PositionFilterMaps/PoseFilterMap.h
        src/PositionFilterMaps/PositionFilterMap.cpp
        src/PositionFilterMaps/PositionFilterMap.h
        src/PositionFilterMaps/PoseChangeFilterMap.cpp
        src/PositionFilterMaps/PoseChangeFilterMap.h
        src/PositionFilterMaps/SWPoseFilterMap.cpp
        src/PositionFilterMaps/SWPoseFilterMap.h
        src/PositionFilterMaps/SWPoseChangeFilterMap.cpp
        src/PositionFilterMaps/SWPoseChangeFilterMap.h
        src/PositionFilterMaps/PositionFilterMapFactory.cpp
        src/PositionFilterMaps/PositionFilterMapFactory.h
        src/DepthEstimation/DepthEstimators/NoFilteringDepthEstimator.cpp
        src/DepthEstimation/DepthEstimators/NoFilteringDepthEstimator.h
        src/CalibrationLoader.cpp
        src/CalibrationLoader.h
        src/CalibrationConfiguration.cpp
        src/CalibrationConfiguration.h)

target_link_libraries(depthest 
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})

#add_dependencies(depthest depthest_generate_messages_cpp)

add_executable(visualizer
        src/vizualizer/Visualizer.cpp
        src/vizualizer/Visualizer3D.cpp
        src/vizualizer/Visualizer3D.h
        src/HelperFunctions.h
        src/vizualizer/DepthVisualizer.h
        src/vizualizer/DepthVisualizer.cpp
        src/Camera.cpp
        src/Camera.h
        src/vizualizer/ReprojectionVisualizer.cpp
        src/vizualizer/ReprojectionVisualizer.h
        src/CalibrationLoader.cpp
        src/CalibrationLoader.h
        src/CalibrationConfiguration.cpp
        src/CalibrationConfiguration.h
        src/Params.cpp
        src/Params.h
        )

target_link_libraries(visualizer ${catkin_LIBRARIES})
#add_dependencies(visualizer visualizer_generate_messages_cpp)

add_executable(remaper
        src/Remapping/Remaper.cpp
        src/Remapping/ImageRemaper.cpp
        src/Remapping/ImageRemaper.h
        src/Camera.cpp
        src/Camera.h
        src/HelperFunctions.h
        src/Remapping/PoseRemaper.cpp
        src/Remapping/PoseRemaper.h
        src/CalibrationLoader.cpp
        src/CalibrationLoader.h
        src/CalibrationConfiguration.cpp
        src/CalibrationConfiguration.h
        src/Params.cpp
        src/Params.h)

target_link_libraries(remaper
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})