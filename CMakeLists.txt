cmake_minimum_required(VERSION 2.8.3)
project(depthest)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,/usr/local/lib")

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
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   visualization_msgs
   geometry_msgs
   std_msgs  # Or other packages containing msgs
   tf2_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(ErrorCalc
	src/Evaluation/ErrorCalc.cpp
)

add_library(Params
	src/Params.cpp
	)

add_library(Camera
	src/Camera.cpp
	src/CalibrationConfiguration.cpp
	src/CalibrationLoader.cpp
)

add_library(Estimators
	src/EstimatorMaps/EstimatorPointMapFactory.cpp
	src/EstimatorMaps/Estimators/MedianEstimator.cpp
	src/EstimatorMaps/MedianEstimatorPointMap.cpp
	src/EstimatorMaps/Estimators/KalmanFilterEstimator.cpp
	src/EstimatorMaps/KalmanFilterEstimatorPointMap.cpp
	src/EstimatorMaps/Estimators/DummyEstimator.cpp
	src/EstimatorMaps/DummyEstimatorPointMap.cpp
	src/EstimatorMaps/Estimators/MedianSWEstimator.cpp
	src/EstimatorMaps/MedianSWEstimatorPointMap.cpp
)

target_link_libraries(Estimators
	${OpenCV_LIBRARIES}
)

add_library(PositionFilter
	src/PositionFilterMaps/PositionFilterMap.cpp
	src/PositionFilterMaps/PoseFilters/PoseFilter.cpp
	src/PositionFilterMaps/PositionFilterMapFactory.cpp
)

add_library(FilterDepthEstimator
	src/DepthEstimation/DepthEstimators/FilterDepthEstimator.cpp
	src/PositionFilterMaps/PoseFilters/SWPoseChangeFilter.cpp
	src/PositionFilterMaps/PoseFilters/PoseChangeFilter.cpp
	src/PositionFilterMaps/PoseFilters/PoseFilter.cpp
	src/PositionFilterMaps/PoseFilters/SWPoseFilter.cpp
	src/PositionFilterMaps/SWPoseChangeFilterMap.cpp
	src/PositionFilterMaps/PoseChangeFilterMap.cpp
	src/PositionFilterMaps/SWPoseFilterMap.cpp
)

target_link_libraries(FilterDepthEstimator
	Estimators
	PositionFilter
	ErrorCalc
	Camera
)

add_executable(depthest
        src/DepthEstimation/Depthes.cpp
	src/DepthEstimation/DepthEstimators/NoFilteringDepthEstimator.cpp
	src/DepthEstimation/DepthEstimatorFactory.cpp
	src/DepthEstimation/NodeHandles/DepthEstimationNode.cpp
)

target_link_libraries(depthest
	FilterDepthEstimator
	Params
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})

add_executable(visualizer
        src/vizualizer/Visualizer.cpp
	src/vizualizer/Visualizer3D.cpp
	src/vizualizer/DepthVisualizer.cpp
	src/vizualizer/ReprojectionVisualizer.cpp
        )

target_link_libraries(visualizer
	Params
	Camera
	${catkin_LIBRARIES}
)

add_executable(remaper
        src/Remapping/Remaper.cpp
	src/Remapping/ImageRemaper.cpp
	src/Remapping/PoseRemaper.cpp
        )

target_link_libraries(remaper
	Camera
	Params
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
