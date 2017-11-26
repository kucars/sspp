cmake_minimum_required(VERSION 2.8.3)
project(sspp)
SET ( CMAKE_CXX_FLAGS "-frounding-math" )
set(CMAKE_BUILD_TYPE Release)

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
  octomap_world
  )

# Few differences when dealing with ros-indigo and ros-kinetic
set( ROS_DISTRO $ENV{ROS_DISTRO} )
if( ${ROS_DISTRO} STREQUAL "indigo" )
    find_package(fcl REQUIRED)
elseif(${ROS_DISTRO} STREQUAL "kinetic")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBFCL REQUIRED fcl)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message("Unsupported ROS Distro")
endif()

find_package(CGAL QUIET COMPONENTS)
find_package(PCL REQUIRED)
find_package(CUDA QUIET)
find_package(octomap REQUIRED)
find_package(component_test QUIET)

add_service_files(
  FILES
  sspp_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

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
   LIBRARIES ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
   CATKIN_DEPENDS roscpp pcl_ros octomap_world
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rviz_visual_tools_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Set include directories
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
    src/sensors.cpp
    )

target_link_libraries(SSPathPlanner
    ${OCTOMAP_LIBRARIES}
    )

add_executable(distance_heuristic_test src/distance_heuristic_test.cpp)
target_include_directories(distance_heuristic_test PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(distance_heuristic_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_heuristic_test SSPathPlanner
    -lgmp -lmpfr
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES}
    )

add_executable(reactive_planner_test src/reactive_planner_test.cpp)
target_include_directories(reactive_planner_test PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(reactive_planner_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(reactive_planner_test SSPathPlanner
    -lgmp -lmpfr
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES}
    )

add_executable(reactive_planner_server_node src/reactive_planner_server_node.cpp src/reactive_planner_server.cpp)
target_include_directories(reactive_planner_server_node PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(reactive_planner_server_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(reactive_planner_server_node SSPathPlanner
    -lgmp -lmpfr
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${rviz_visual_tools_LIBRARIES}
    )

if (component_test_FOUND)
        add_library(CoverageHeuristic SHARED src/coverage_path_planning_heuristic.cpp)
        target_link_libraries(CoverageHeuristic ${component_test_LIBRARIES} occlusion_culling_gpu mesh_surface ${OCTOMAP_LIBRARIES})
        target_include_directories(CoverageHeuristic PUBLIC include ${catkin_INCLUDE_DIRS} ${component_test_INCLUDE_DIRS} /usr/local/cuda/include/)
        add_executable(coverage_heuristic_test src/coverage_heuristic_test.cpp)
        target_include_directories(coverage_heuristic_test PUBLIC include ${catkin_INCLUDE_DIRS} ${component_test_INCLUDE_DIRS} /usr/local/cuda/include/)
        target_link_libraries(coverage_heuristic_test SSPathPlanner
            -lgmp -lmpfr
            ${CGAL_LIBRARY}
            ${CGAL_Core_LIBRARY}
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OCTOMAP_LIBRARIES}
            ${component_test_LIBRARIES}
            occlusion_culling_gpu
            ${rviz_visual_tools_LIBRARIES}
            CoverageHeuristic
            )

        add_executable(coverage_multi_agent_test src/coverage_multi_agent_test.cpp)
        target_include_directories(coverage_multi_agent_test PUBLIC include ${catkin_INCLUDE_DIRS} ${component_test_INCLUDE_DIRS} /usr/local/cuda/include/)
        target_link_libraries(coverage_multi_agent_test SSPathPlanner
            -lgmp -lmpfr
            ${CGAL_LIBRARY}
            ${CGAL_Core_LIBRARY}
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OCTOMAP_LIBRARIES}
            ${component_test_LIBRARIES}
            occlusion_culling_gpu
            ${rviz_visual_tools_LIBRARIES}
            CoverageHeuristic
            )
endif (component_test_FOUND)
