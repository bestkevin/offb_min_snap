cmake_minimum_required(VERSION 3.0.2)
project(min_snap_trajectory)

# 查找必需的CMake和Catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  polynomial_trajectories
  quadrotor_common
)

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 声明Catkin包和依赖项
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    geometry_msgs 
    mavros_msgs 
    polynomial_trajectories 
    quadrotor_common
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 声明安装路径（确保头文件可被其他包引用）
install(DIRECTORY include/${offb_min_snap}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 添加可执行文件
add_executable(offb_min_snap_node /home/erb110/catkin_ws/src/offb_min_snap/src/offb_min_snap/offb_min_snap.cpp)

# 链接库
target_link_libraries(offb_min_snap_node
  ${catkin_LIBRARIES}
  stdc++fs
  zmq
  zmqpp
)
