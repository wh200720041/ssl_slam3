cmake_minimum_required(VERSION 2.8.3)
project(ssl_slam3)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  octomap_msgs
  octomap_ros
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
)

find_package(Eigen3)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Ceres 
  INCLUDE_DIRS include
)

add_executable(ssl_slam3_laser_processing_node src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/param.cpp src/utils.cpp)
target_link_libraries(ssl_slam3_laser_processing_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(ssl_slam3_odom_estimation_node src/utils.cpp src/odomEstimationNode.cpp src/param.cpp src/odomEstimationClass.cpp src/imuOptimizationFactor.cpp src/lidarOptimizationFactor.cpp src/imuPreintegrationClass.cpp src/param.cpp src/poseOptimizationFactor.cpp)
target_link_libraries(ssl_slam3_odom_estimation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(ssl_slam3_laser_mapping_node src/laserMappingNode.cpp src/utils.cpp src/laserMappingClass.cpp src/param.cpp)
target_link_libraries(ssl_slam3_laser_mapping_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})
