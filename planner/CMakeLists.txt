cmake_minimum_required(VERSION 2.8.3)

project(gcopter)

set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  tf
  sensor_msgs
  nav_msgs
  visualization_msgs
  plan_env
  cv_bridge
)
message(WARNING "!!!!!! ${PROJECT_SOURCE_DIR}")
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES gcopter
  CATKIN_DEPENDS plan_env
)

add_executable(global_planning src/global_planning.cpp)

target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(traj_server_node src/traj_server.cpp)
target_link_libraries(traj_server_node ${catkin_LIBRARIES})

add_executable(traj_server_transport_node src/traj_server_transport.cpp)
target_link_libraries(traj_server_transport_node ${catkin_LIBRARIES})
