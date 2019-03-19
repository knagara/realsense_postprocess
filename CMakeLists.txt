cmake_minimum_required(VERSION 2.8.3)
project(realsense_postprocess)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs geometry_msgs eigen_conversions
)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs geometry_msgs eigen_conversions
)

add_executable(realsense_postprocess_cloud_filter src/cloud_filter.cpp)
target_link_libraries(realsense_postprocess_cloud_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
