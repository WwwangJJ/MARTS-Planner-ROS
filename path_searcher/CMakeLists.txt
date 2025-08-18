cmake_minimum_required(VERSION 2.8.3)

project(grid_path_searcher)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    tf
    std_msgs
    nav_msgs
    quadrotor_msgs
    visualization_msgs 
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror
set( CMAKE_CXX_STANDARD 14)

##生成astar_path_searcher节点
add_executable( astar_path_searcher
    src/astar_ros.cpp
    src/Astar.cpp
    )
target_link_libraries(astar_path_searcher 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

##生成complex_map点云地图节点
add_executable ( complex_map 
    src/complex_map.cpp )
target_link_libraries( complex_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} ) 

##生成waypoint_generator节点
add_executable ( waypoint_generator 
    src/waypoint_generator.cpp )
target_link_libraries( waypoint_generator
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} ) 