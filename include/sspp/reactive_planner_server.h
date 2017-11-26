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
  volumetric_mapping::OctomapManager * mapManager = NULL;
  rviz_visual_tools::RvizVisualToolsPtr visualTools;
  geometry_msgs::Point robotCenter;
  geometry_msgs::Vector3 gridSize;
  geometry_msgs::Pose gridStart;
  SSPP::PathPlanner* pathPlanner = NULL;
  Robot* robot = NULL;
  Pose start,end;
  double orientationSamplingRes = 90.0;
  double debugDelay = 0.0;
  double regGridConRad;
  double gridRes;
  double distanceToGoal = 0;
  int treeProgressDisplayFrequency = -1;
  bool gotCloud = false;
  bool donePlanning = false;
  bool visualizeSearchSpace = false;
  bool sampleOrientations = false;
  bool debug = false;
  std::vector<std::pair<Eigen::Vector3d, double> > occupied_box_vector;
public:
  ReactivePlannerServer(const ros::NodeHandle& nh_, const ros::NodeHandle& nh_private_, volumetric_mapping::OctomapManager * mapManager_);
  ~ReactivePlannerServer();
  bool plannerCallback(sspp::sspp_srv::Request &req, sspp::sspp_srv::Response &res);
  void callback(const sensor_msgs::PointCloud2::ConstPtr &cloudIn);
  void getConfigsFromRosParams();
};
