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
)
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_calibration_odom_laser.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_cli.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_csparse_extension.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_ext_csparse.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_ext_freeglut_minimal.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_hierarchical.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_incremental.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_interactive.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_interface.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_opengl_helper.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_parser.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_simulator.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_csparse.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_eigen.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_pcg.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_slam2d_linear.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_structure_only.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_tutorial_slam2d.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_tutorial_slam3d.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_data.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_icp.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sclam2d.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam2d.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam2d_addons.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d_addons.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_viewer.so
    ${PCL_LIBRARIES}
)

add_executable(gyro_odometry src/gyro_odometry.cpp)
target_link_libraries(gyro_odometry
    ${catkin_LIBRARIES}
)

add_executable(data_save src/data_save.cpp)
target_link_libraries(data_save
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(rm_cluster src/rm_cluster.cpp)
target_link_libraries(rm_cluster
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(transformer src/transform.cpp)
target_link_libraries(transformer
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(gicp src/gicp.cpp)
target_link_libraries(gicp
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(slam3d Examples/slam3d.cpp)
target_link_libraries(slam3d
     ${PROJECT_NAME}
     )

