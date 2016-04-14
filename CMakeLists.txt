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
  rviz_visual_tools
  )

find_package( CGAL QUIET COMPONENTS  )
find_package(fcl REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA QUIET)
find_package(component_test QUIET)

if ( NOT CUDA_FOUND )
  message(STATUS "You will not be able to use coverage heuristic, it requires CUDA")
#  return()
endif()

if ( NOT component_test_FOUND )
  message(STATUS "You will not be able to use coverage heuristic, it requires component test package")
#  return()
endif()

include(FindPkgConfig)

#catkin_package(CATKIN_DEPENDS component_test)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp pcl_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
)

#pkg_check_modules(PLAYER REQUIRED playercore playerc++ playerc)

##################
#    QT STUFF    #
##################
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt4 REQUIRED)

# Set include directories
include_directories(include ${QT_INCLUDES})
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
    src/distance_heuristic.cpp
    src/rviz_drawing_tools.cpp
    )

target_link_libraries(SSPathPlanner
    Qt4::QtGui
    Qt4::QtCore
    )
add_executable(distance_heuristic_test src/distance_heuristic_test.cpp)
target_include_directories(distance_heuristic_test PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(distance_heuristic_test SSPathPlanner
    -lgmp -lmpfr
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    Qt4::QtGui
    Qt4::QtCore
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES}
    )

if (component_test_FOUND)
        add_library(CoverageHeuristic SHARED src/coverage_path_planning_heuristic.cpp)
        target_link_libraries(CoverageHeuristic Qt4::QtGui Qt4::QtCore ${component_test_LIBRARIES} occlusion_culling_gpu)
        target_include_directories(CoverageHeuristic PUBLIC include ${catkin_INCLUDE_DIRS} ${component_test_INCLUDE_DIRS} /usr/local/cuda/include/)
        add_executable(coverage_heuristic_test src/coverage_heuristic_test.cpp)
        target_include_directories(coverage_heuristic_test PUBLIC include ${catkin_INCLUDE_DIRS} ${component_test_INCLUDE_DIRS} /usr/local/cuda/include/)
        target_link_libraries(coverage_heuristic_test SSPathPlanner
            -lgmp -lmpfr
            ${CGAL_LIBRARY}
            ${CGAL_Core_LIBRARY}
            Qt4::QtGui
            Qt4::QtCore
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${component_test_LIBRARIES}
            occlusion_culling_gpu
            ${rviz_visual_tools_LIBRARIES}
            CoverageHeuristic
            )
endif (component_test_FOUND)
