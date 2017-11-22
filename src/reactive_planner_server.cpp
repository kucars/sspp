/***************************************************************************
 *   Copyright (C) 2006 - 2017 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include <ros/ros.h>
#include "sspp/pathplanner.h"

#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "sspp/distance_heuristic.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include <octomap_world/octomap_manager.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sspp/sspp_srv.h>

class ReactivePlannerServer
{
private:
  ros::ServiceServer planningService;
  ros::Subscriber sub;
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  volumetric_mapping::OctomapManager * manager = NULL;
  rviz_visual_tools::RvizVisualToolsPtr visualTools;
  SSPP::PathPlanner* pathPlanner = NULL;
  Robot* robot = NULL;
  geometry_msgs::Point robotCenter;
  bool gotCloud = false;
  bool donePlanning = false;
  bool visualizeSearchSpace = false;
  bool sampleOrientations = false;
  bool debug = false;
  Pose start,end;
  double orientationSamplingRes = 90.0;
  double debugDelay = 0.0;
  double regGridConRad;
  double gridRes;
  double distanceToGoal = 0;
  int treeProgressDisplayFrequency = -1;
  geometry_msgs::Vector3 gridSize;
  geometry_msgs::Pose gridStart;
  std::vector<std::pair<Eigen::Vector3d, double> > occupied_box_vector;
public:
  ReactivePlannerServer(const ros::NodeHandle& nh_, const ros::NodeHandle& nh_private_):
    nh(nh_),
    nh_private(nh_private_)
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>("cloud_pcd", 1, &ReactivePlannerServer::callback,this);
    planningService = nh.advertiseService("sspp_planner", &ReactivePlannerServer::plannerCallback, this);
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/sspp_visualisation"));
    visualTools->loadMarkerPub();
    gridStart.position.x = 0.0;
    gridStart.position.y = 0.0;
    gridStart.position.z = 0.0;

    getConfigsFromRosParams();
    visualTools->deleteAllMarkers();
    visualTools->enableBatchPublishing();
    manager = new volumetric_mapping::OctomapManager(nh, nh_private);
    Eigen::Vector3d origin(0,0,0);
    Eigen::Vector3d envBox(15,15,10);
    manager->setFree(origin,envBox);
    ROS_INFO("Starting the reactive planning");
  }

  ~ReactivePlannerServer()
  {
    if(robot)
      delete robot;
    if(pathPlanner)
      delete pathPlanner;
    if (manager)
      delete manager;
  }

  bool plannerCallback(sspp::sspp_srv::Request& req, sspp::sspp_srv::Response& res)
  {
    manager->getAllOccupiedBoxes(&occupied_box_vector);
    ROS_INFO_THROTTLE(1,"Received a Service Request Planning MAP SIZE:[%f %f %f] occupied cells:%lu",manager->getMapSize()[0],manager->getMapSize()[1],manager->getMapSize()[2],occupied_box_vector.size());

    if(!gotCloud || occupied_box_vector.size()<=0 || manager->getMapSize().norm() <= 0.0)
    {
        ROS_INFO_THROTTLE(1, "Planner not set up: Octomap is empty!");
        return false;
    }

    ros::Time timer_start = ros::Time::now();

    start.p   = req.start;
    end.p     = req.end;
    gridStart = req.grid_start;

    visualTools->publishSphere(start.p, rviz_visual_tools::BLUE, 0.3,"start_pose");
    visualTools->publishSphere(end.p, rviz_visual_tools::ORANGE, 0.3,"end_pose");
    visualTools->trigger();

    double robotH = 0.9, robotW = 0.5, narrowestPath = 0.987;

    robotCenter.x = -0.3f;
    robotCenter.y = 0.0f;
    if(!robot)
      robot = new Robot("Robot", robotH, robotW, narrowestPath, robotCenter);

    //Every how many iterations to display the tree; -1 disable display
    if(!pathPlanner)
      pathPlanner = new SSPP::PathPlanner(nh, robot, regGridConRad, treeProgressDisplayFrequency);

    // This causes the planner to pause for the desired amount of time and display
    // the search tree, useful for debugging
    pathPlanner->setDebugDelay(debugDelay);

    SSPP::DistanceHeuristic distanceHeuristic(nh, debug,manager,visualTools);
    distanceHeuristic.setEndPose(end.p);
    distanceHeuristic.setTolerance2Goal(distanceToGoal);
    pathPlanner->setHeuristicFucntion(&distanceHeuristic);

    // Generate Grid Samples and visualise it
    pathPlanner->generateRegularGrid(gridStart, gridSize, gridRes, sampleOrientations, orientationSamplingRes,false, true);
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();

    std::vector<geometry_msgs::PoseArray> sensorsPoseSS;
    geometry_msgs::PoseArray robotPoseSS;
    pathPlanner->getRobotSensorPoses(robotPoseSS, sensorsPoseSS);
    std::cout << "\n\n---->>> Total Nodes in search Space ="<< searchSpaceNodes.size();

    visualTools->publishSpheres(searchSpaceNodes, rviz_visual_tools::PURPLE, 0.1,"search_space_nodes");
    visualTools->trigger();

    // Connect nodes and visualise it
    pathPlanner->connectNodes();
    std::cout << "\nSpace Generation took:"
              << double(ros::Time::now().toSec() - timer_start.toSec())
              << " secs";
    if(visualizeSearchSpace)
    {
      std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
      for(int i =0; i<(searchSpaceConnections.size() - 1) ;i+=2)
      {
        visualTools->publishLine(searchSpaceConnections[i], searchSpaceConnections[i+1], rviz_visual_tools::BLUE,rviz_visual_tools::LARGE);
      }
      visualTools->trigger();
    }

    // Find path and visualise it
    ros::Time timer_restart = ros::Time::now();
    SSPP::Node* path = pathPlanner->startSearch(start);
    ros::Time timer_end = ros::Time::now();
    std::cout << "\nPath Finding took:" << double(timer_end.toSec() - timer_restart.toSec()) << " secs";

    if (path)
    {
      pathPlanner->printNodeList();
    }
    else
    {
      std::cout << "\nNo Path Found";
      return false;
    }

    geometry_msgs::Point linePoint;
    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose, sensorPose,pathPoses;
    double dist = 0;
    double yaw;
    while (path != NULL)
    {
      tf::Quaternion qt(path->pose.p.orientation.x, path->pose.p.orientation.y,
                        path->pose.p.orientation.z, path->pose.p.orientation.w);
      yaw = tf::getYaw(qt);
      pathPoses.poses.push_back(path->pose.p);
      if (path->next != NULL)
      {
        linePoint.x = path->pose.p.position.x;
        linePoint.y = path->pose.p.position.y;
        linePoint.z = path->pose.p.position.z;
        robotPose.poses.push_back(path->pose.p);
        for (int i = 0; i < path->senPoses.size(); i++)
          sensorPose.poses.push_back(path->senPoses[i].p);
        pathSegments.push_back(linePoint);

        linePoint.x = path->next->pose.p.position.x;
        linePoint.y = path->next->pose.p.position.y;
        linePoint.z = path->next->pose.p.position.z;
        robotPose.poses.push_back(path->next->pose.p);
        for (int i = 0; i < path->next->senPoses.size(); i++)
          sensorPose.poses.push_back(path->next->senPoses[i].p);
        pathSegments.push_back(linePoint);

        dist = dist + Dist(path->next->pose.p, path->pose.p);
      }
      path = path->next;
    }

    res.path = pathPoses.poses;

    std::cout << "\nDistance calculated from the path: " << dist << "m\n";

    for(int i =0; i<(pathSegments.size() - 1) ;i++)
    {
      visualTools->publishLine(pathSegments[i], pathSegments[i+1], rviz_visual_tools::RED);
    }
    visualTools->trigger();

    for (int i = 0; i < robotPose.poses.size(); i++)
    {
      visualTools->publishArrow(robotPose.poses[i], rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE, 0.3);
    }
    visualTools->trigger();

    for (int i = 0; i < robotPoseSS.poses.size(); i++)
    {
      visualTools->publishArrow(robotPoseSS.poses[i], rviz_visual_tools::CYAN, rviz_visual_tools::LARGE, 0.3);
    }
    visualTools->trigger();

    for (int i = 0; i < sensorsPoseSS.size(); i++)
    {
      for(int j = 0; j < sensorsPoseSS[i].poses.size(); j++)
        visualTools->publishArrow(sensorsPoseSS[i].poses[j], rviz_visual_tools::DARK_GREY, rviz_visual_tools::LARGE, 0.3);
    }
    visualTools->trigger();
    return true;
  }

  void getConfigsFromRosParams()
  {
    std::string ns = ros::this_node::getName();
    std::cout<<"Node name is:"<<ns<<"\n";
    nh_private.param("start_x",start.p.position.x,start.p.position.x);
    nh_private.param("start_y",start.p.position.y,start.p.position.y);
    nh_private.param("start_z",start.p.position.z,start.p.position.z);
    nh_private.param("end_x",end.p.position.x,end.p.position.x);
    nh_private.param("end_y",end.p.position.y,end.p.position.y);
    nh_private.param("end_z",end.p.position.z,end.p.position.z);
    nh_private.param("connection_rad",regGridConRad,regGridConRad);
    nh_private.param("grid_resolution",gridRes,gridRes);
    nh_private.param("grid_size_x",gridSize.x,gridSize.x);
    nh_private.param("grid_size_y",gridSize.y,gridSize.y);
    nh_private.param("grid_size_z",gridSize.z,gridSize.z);
    nh_private.param("grid_start_x",gridStart.position.x,gridStart.position.x);
    nh_private.param("grid_start_y",gridStart.position.y,gridStart.position.y);
    nh_private.param("grid_start_z",gridStart.position.z,gridStart.position.z);
    nh_private.param("visualize_search_space",visualizeSearchSpace,visualizeSearchSpace);
    nh_private.param("debug",debug,debug);
    nh_private.param("debug_delay",debugDelay,debugDelay);
    nh_private.param("dist_to_goal",distanceToGoal,distanceToGoal);
    nh_private.param("sample_orientations",sampleOrientations,sampleOrientations);
    nh_private.param("orientation_sampling_res",orientationSamplingRes,orientationSamplingRes);
    nh_private.param("tree_progress_display_freq",treeProgressDisplayFrequency,treeProgressDisplayFrequency);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& cloudIn)
  {
    gotCloud = true;
    manager->insertPointcloudWithTf(cloudIn);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reactive_planner_server");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ReactivePlannerServer reactivePlannerServer(nh,nh_private);
  ros::spin();
  return 0;
}
