cmake_minimum_required(VERSION 3.0.2)
project(pcl_sample)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  image_transport
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)
catkin_package(

)
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( include ${PCL_INCLUDE_DIRS} )
include_directories("/usr/include/eigen3")


add_executable(pcd_write src/pcd_write.cpp)
target_link_libraries(pcd_write ${catkin_LIBRARIES} )


add_executable(plane_segmentation src/plane_segmentation.cpp)
target_link_libraries(plane_segmentation ${catkin_LIBRARIES} )


add_executable(plane src/plane.cpp)
target_link_libraries(plane ${catkin_LIBRARIES} )

add_executable(through src/through.cpp)
target_link_libraries(through ${catkin_LIBRARIES} )

add_executable(select_plane src/select_plane.cpp)
target_link_libraries(select_plane ${catkin_LIBRARIES} )
