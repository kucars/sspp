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

class ReactivePlanner
{
private:
  ros::Subscriber sub;
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  volumetric_mapping::OctomapManager * manager;
  rviz_visual_tools::RvizVisualToolsPtr visualTools;

public:
  ReactivePlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
    nh(nh),
    nh_private(nh_private)
  {
    sub = nh.subscribe("cloud_pcd", 1, &ReactivePlanner::gotCloud,this);
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/sspp_visualisation"));
    visualTools->loadMarkerPub();

    ROS_INFO("Sleeping 5 seconds before running demo");
    //ros::Duration(5.0).sleep();
    visualTools->deleteAllMarkers();
    visualTools->enableBatchPublishing();
    manager = new volumetric_mapping::OctomapManager(nh, nh_private);
  }

  void gotCloud(const sensor_msgs::PointCloud2::ConstPtr& cloudIn)
  {
    manager->insertPointcloudWithTf(cloudIn);
    ros::Time timer_start = ros::Time::now();
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;
    gridStartPose.position.x = 0.0;
    gridStartPose.position.y = 0.0;
    gridStartPose.position.z = 0.0;
    gridSize.x = 20.0;
    gridSize.y = 20.0;
    gridSize.z = 2.0;
    double gridRes = 1.0;

    SSPP::PathPlanner* pathPlanner;
    Pose start(0.0, 0.0, 0, DTOR(0.0));
    Pose end(8.0, 8.0, 2.0, DTOR(0.0));

    visualTools->publishSphere(start.p, rviz_visual_tools::BLUE, 0.3,"start_pose");
    visualTools->publishSphere(end.p, rviz_visual_tools::ORANGE, 0.3,"end_pose");
    visualTools->trigger();

    double robotH = 0.9, robotW = 0.5, narrowestPath = 0.987;
    double distanceToGoal = 1.2, regGridConRad = 1.5;

    geometry_msgs::Point robotCenter;
    robotCenter.x = -0.3f;
    robotCenter.y = 0.0f;
    Robot* robot = new Robot("Robot", robotH, robotW, narrowestPath, robotCenter);

    //Every how many iterations to display the tree; -1 disable display
    int progressDisplayFrequency = -1;
    pathPlanner = new SSPP::PathPlanner(nh, robot, regGridConRad, progressDisplayFrequency);

    // This causes the planner to pause for the desired amount of time and display
    // the search tree, useful for debugging
    pathPlanner->setDebugDelay(0.0);

    SSPP::DistanceHeuristic distanceHeuristic(nh, false);
    distanceHeuristic.setEndPose(end.p);
    distanceHeuristic.setTolerance2Goal(distanceToGoal);
    pathPlanner->setHeuristicFucntion(&distanceHeuristic);

    // Generate Grid Samples and visualise it
    pathPlanner->generateRegularGrid(gridStartPose, gridSize, gridRes, true, 90.0f,false, true);
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
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();

    //visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE, "search_space");
    //visualTools->trigger();

    // Find path and visualise it
    ros::Time timer_restart = ros::Time::now();
    SSPP::Node* path = pathPlanner->startSearch(start);
    ros::Time timer_end = ros::Time::now();
    std::cout << "\nPath Finding took:" << double(timer_end.toSec() - timer_restart.toSec()) << " secs";

    // path print and visualization
    if (path)
    {
      pathPlanner->printNodeList();
    }
    else
    {
      std::cout << "\nNo Path Found";
    }

    geometry_msgs::Point linePoint;
    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose, sensorPose;
    double dist = 0;
    double yaw;
    while (path != NULL)
    {
      tf::Quaternion qt(path->pose.p.orientation.x, path->pose.p.orientation.y,
                        path->pose.p.orientation.z, path->pose.p.orientation.w);
      yaw = tf::getYaw(qt);
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

    delete robot;
    delete pathPlanner;
    if (manager)
    {
      delete manager;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reactive_planner_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ReactivePlanner reactivePlanner(nh,nh_private);
  ros::spin();
  return 0;
}
