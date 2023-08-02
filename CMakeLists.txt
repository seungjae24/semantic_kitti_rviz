cmake_minimum_required(VERSION 3.17)
project(semantic_kitti_rviz)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        cv_bridge
        pcl_conversions
        pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

add_executable(semantic_kitti_rviz src/semantic_kitti_rviz.cpp)
target_link_libraries(semantic_kitti_rviz ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)

add_executable(nuscenes_rviz src/nuscenes_rviz.cpp)
target_link_libraries(nuscenes_rviz ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)