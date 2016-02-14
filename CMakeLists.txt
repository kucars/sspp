cmake_minimum_required(VERSION 2.8.3)
project(sspp)
SET ( CMAKE_CXX_FLAGS "-frounding-math" )

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
#find_package(component_test REQUIRED)
find_package( CGAL QUIET COMPONENTS  )
find_package(fcl REQUIRED)

include(FindPkgConfig)
  
catkin_package(CATKIN_DEPENDS component_test)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${component_test_INCLUDE_DIRS}
)

pkg_check_modules(PLAYER REQUIRED playercore playerc++ playerc)

##################
#    QT STUFF    #
##################

ADD_DEFINITIONS( ${QT_DEFINITIONS} )
FIND_PACKAGE( Qt4 REQUIRED )

SET(QT_USE_QTGUI TRUE )
SET(QT_USE_QTOPENGL TRUE )
SET(QT_USE_XML TRUE )
SET(QT_USE_QTXMLPATTERNS TRUE)



INCLUDE( ${QT_USE_FILE} )

# Set include directories
include_directories(include ${QT_INCLUDES} ${PLAYER_INCLUDE_DIRS} ${component_test_INCLUDE_DIRS})
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
    src/configfile.cc
    src/ssppexception.cpp
    src/heuristic.cpp
    )
add_executable(ssppTest src/test.cpp)
target_include_directories(ssppTest PUBLIC include ${catkin_INCLUDE_DIRS} ${GTK_INCLUDE_DIRS} ${component_test_INCLUDE_DIRS})
target_link_libraries( ssppTest SSPathPlanner
    -lgmp -lmpfr ${CGAL_LIBRARY} ${CGAL_Core_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
    ${QT_QTXML_LIBRARY}
    ${QT_LIBRARIES}
    ${PLAYER_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${component_test_LIBRARIES}
    )
