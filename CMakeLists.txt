## 
cmake_minimum_required(VERSION 3.0.0)
project(ampt_os)
set(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_srvs
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters visualization)

catkin_package(
   CATKIN_DEPENDS  message_runtime
)

include_directories(
${PCL_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
src/cpp_solver
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_subdirectory(src/cpp_solver)

add_executable( ampt_main
  src/main.cpp
  src/TopoMetric.cpp
  src/Decision.cpp
  src/RCPP.cpp
  src/TopoSim.cpp
  src/cmpalg/ViterbiLoc.cpp
  src/cmpalg/ActLocBaseline.cpp
  src/cmpalg/ActLocOverlay.cpp
)
add_dependencies( ampt_main
  cpp_solver_library
)
target_link_libraries( ampt_main
  cpp_solver_library
  ${catkin_LIBRARIES}
  ${EIGEN_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
