cmake_minimum_required(VERSION 2.8.3)
project(graph_slam)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(PCL 1.8 QUIET)
if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL > 1.8 not found.")
endif()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAG "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenGL)

include_directories(
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/EXTERNAL/csparse
    ${PROJECT_SOURCE_DIR}/Thirdparty
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_LIBRARIES}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES graph_slam
#  CATKIN_DEPENDS geoemtry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_library(${PROJECT_NAME} SHARED
    src/G2O.cpp
    src/System.cpp
    src/Gicp.cpp
    src/Util.cpp
    src/File.cpp
    src/PCL.cpp
    src/ICP.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_csparse_extension.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_csparse.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d_addons.so
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
)

add_executable(gyro_odometry Examples/ROS/gyro_odometry.cpp)
target_link_libraries(gyro_odometry
    ${catkin_LIBRARIES}
)

add_executable(saver Examples/ROS/saver.cpp)
target_link_libraries(saver
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(rm_cluster Examples/ROS/rm_cluster.cpp)
target_link_libraries(rm_cluster
    ${PROJECT_NAME}
)

add_executable(integrator Examples/ROS/integrator.cpp)
target_link_libraries(integrator
    ${PROJECT_NAME}
)


add_executable(node_edge Examples/SLAM/node_edge.cpp)
target_link_libraries(node_edge
    ${PROJECT_NAME}
)



add_executable(transformer src/transform.cpp)
target_link_libraries(transformer
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(slam3d Examples/SLAM/slam3d.cpp)
target_link_libraries(slam3d
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)

add_executable(gicp_test Examples/Test/gicp_test.cpp)
target_link_libraries(gicp_test
    ${PCL_LIBRARIES}
)

add_executable(node_test Examples/Test/node_test.cpp)
target_link_libraries(node_test
    ${PCL_LIBRARIES}
    ${PROJECT_NAME}
)

add_executable(test_v2 Examples/Test/test_v2.cpp)
target_link_libraries(test_v2
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(test_v3 Examples/Test/test_v3.cpp)
target_link_libraries(test_v3
  ${PROJECT_NAME}
)

add_executable(node_edge_v3 Examples/Test/node_edge_v3.cpp)
target_link_libraries(node_edge_v3
  ${PROJECT_NAME}
)
