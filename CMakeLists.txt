cmake_minimum_required(VERSION 2.8.3)
project(sspp)
SET ( CMAKE_CXX_FLAGS "-frounding-math" )
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  shape_msgs
  std_msgs
  component_test
  )

find_package( CGAL QUIET COMPONENTS  )
find_package(fcl REQUIRED)
find_package(CUDA REQUIRED)

include(FindPkgConfig)

catkin_package(CATKIN_DEPENDS component_test)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${component_test_INCLUDE_DIRS}
  /usr/local/cuda/include/
)

#pkg_check_modules(PLAYER REQUIRED playercore playerc++ playerc)

##################
#    QT STUFF    #
##################
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 REQUIRED)

# Set include directories
include_directories(include ${QT_INCLUDES} ${component_test_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(SSPathPlanner SHARED
    src/astar.cpp
    src/llist.cpp
    src/map.cpp
    src/node.cpp
    src/pathplanner.cpp
    src/robot.cpp
    src/searchspace.cpp
    src/searchspacenode.cpp
    src/ssppexception.cpp
    src/coverage_path_planning_heuristic.cpp
    src/distance_heuristic.cpp
    )
target_link_libraries(SSPathPlanner
    Qt4::QtGui
    Qt4::QtCore
    )
add_executable(ssppTest src/test.cpp)
target_include_directories(ssppTest PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries( ssppTest SSPathPlanner
    -lgmp -lmpfr
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    Qt4::QtGui
    Qt4::QtCore
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${component_test_LIBRARIES}
    occlusion_culling_gpu
    )
