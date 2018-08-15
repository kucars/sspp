/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
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
#include "sspp/heuristic_interface.h"
#include "sspp/distance_heuristic.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace SSPP
{

DistanceHeuristic::DistanceHeuristic(ros::NodeHandle & nh, bool d,volumetric_mapping::OctomapManager * manager_,rviz_visual_tools::RvizVisualToolsPtr visualTools_):
  debug(d),
  manager(manager_),
  visualTools(visualTools_)
{
    treePub = nh.advertise<visualization_msgs::Marker>("search_tree", 10);
    pathPointPub = nh.advertise<visualization_msgs::Marker>("path_point" , 10);
    pathPub = nh.advertise<visualization_msgs::Marker>("path_testing", 10);
}

double DistanceHeuristic::gCost(Node *node)
{
    double cost;
    if(node == NULL || node->parent==NULL)
        return 0.0;
    cost = node->parent->g_value + Dist(node->pose.p,node->parent->pose.p);
/*
    geometry_msgs::Pose startPose;
    startPose.position.x = 0;
    startPose.position.y = 0;
    startPose.position.z = 0;
    cost = Dist(node->pose.p,startPose);
*/
    return cost;
}

double DistanceHeuristic::hCost(Node *node)
{
    double h=0;
    if(node == NULL)
        return(0);
    // Using the Euclidean distance
    h = Dist(endPose,node->pose.p);
    return h;
}

bool DistanceHeuristic::terminateConditionReached(Node *node)
{
    double deltaDist;
    deltaDist = Dist(node->pose.p,endPose);

    if(debug)
    {
      geometry_msgs::Point linept;
      linept.x = node->pose.p.position.x; linept.y = node->pose.p.position.y; linept.z = node->pose.p.position.z;
      points.push_back(linept);
      visualization_msgs::Marker pointsList = drawPoints(points,3,10000);
      pathPointPub.publish(pointsList);

      if(node->parent)
      {
        linepoint.x = node->pose.p.position.x; linepoint.y =  node->pose.p.position.y; linepoint.z = node->pose.p.position.z;
        lines.push_back(linepoint);
        linepoint.x = node->parent->pose.p.position.x; linepoint.y = node->parent->pose.p.position.y; linepoint.z = node->parent->pose.p.position.z;
        lines.push_back(linepoint);
        visualization_msgs::Marker linesList = drawLines(lines,333333,1,1000000,0.2);
        pathPub.publish(linesList);
      }

      std::cout<<"\n\n ****************** Delta distance %: "<<deltaDist <<" **************************"<<std::endl;
      std::cout<<"\n\n ****************** node pose : ( "<<node->pose.p.position.x <<", "<<node->pose.p.position.y<<", "<<node->pose.p.position.z<<" )**************************"<<std::endl;
    }
    if ( deltaDist <= tolerance2Goal)
        return true;
    else
        return false;
}

bool DistanceHeuristic::isConnectionConditionSatisfied(geometry_msgs::Pose temp, geometry_msgs::Pose S)
{
  if(manager)
  {
    bool noCollision = true;
    Eigen::Vector3d startPoint(temp.position.x, temp.position.y, temp.position.z);
    Eigen::Vector3d endPoint(S.position.x, S.position.y, S.position.z);
    //TODO: read size of robot from params
    Eigen::Vector3d boundingBox(0.5,0.5,0.3);
    volumetric_mapping::OctomapManager::CellStatus cellStatus;
    cellStatus = manager->getLineStatusBoundingBox(startPoint,endPoint,boundingBox);

    if(cellStatus == volumetric_mapping::OctomapManager::CellStatus::kFree)
      noCollision = true;
    else
      noCollision = false;
    if(debug && visualTools)
    {
      if(!noCollision)
        visualTools->publishLine(startPoint,endPoint,rviz_visual_tools::RED, rviz_visual_tools::LARGE);
      else
        visualTools->publishLine(startPoint,endPoint,rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
      visualTools->trigger();
    }

    return noCollision;
  }
  return true;
}

bool DistanceHeuristic::isConnectionConditionSatisfied(SearchSpaceNode *temp, SearchSpaceNode *S)
{
  return isConnectionConditionSatisfied(temp->location,S->location);
}

bool DistanceHeuristic::isFilteringConditionSatisfied(geometry_msgs::Pose pose, geometry_msgs::PoseArray& correspondingSensorPoses, double minDist, double maxDist, pcl::PointCloud<pcl::PointXYZ>& globalCloud, std::vector<pcl::PointCloud<pcl::PointXYZ> >& accuracyClusters, double accuracyThreshhold)
{
    //TODO::preform filtering check
    return true;
}

double DistanceHeuristic::pointCloudDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudDiffPtr )
{
    //TODO::Finding a way to not include this function here too, it is for coverage heuristic
    return 1;
}
void DistanceHeuristic::clusteringPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> >& clustersPointCloudVec, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudDiffPtr)
{
    //TODO::Finding a way to not include this function here too, it is for coverage heuristic
    return;
}
void DistanceHeuristic::findClusterOuterPoints(geometry_msgs::PoseArray waypoints, pcl::PointCloud<pcl::PointXYZ>& cloudHull)
{
    //TODO::Finding a way to not include this function here too, it is for coverage heuristic
    return;
}
void DistanceHeuristic::findClusterBB(pcl::PointCloud<pcl::PointXYZ> clusterPoints, geometry_msgs::Vector3 &gridSize, geometry_msgs::Pose &gridStart)
{
    //TODO::Finding a way to not include this function here too, it is for coverage heuristic
    return;
}

void DistanceHeuristic::displayProgress(vector<Tree> tree)
{
    geometry_msgs::Pose child;
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint1,linePoint2;
    for(unsigned int k=0;k<tree.size();k++)
    {
        for(int j=0;j<tree[k].children.size();j++)
        {
            child = tree[k].children[j];

            linePoint1.x = tree[k].location.position.x;
            linePoint1.y = tree[k].location.position.y;
            linePoint1.z = tree[k].location.position.z;
            lineSegments.push_back(linePoint1);

            linePoint2.x = child.position.x;
            linePoint2.y = child.position.y;
            linePoint2.z = child.position.z;
            lineSegments.push_back(linePoint2);
            if(visualTools)
              visualTools->publishLine(linePoint1,linePoint2,rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
        }
        if(visualTools)
          visualTools->trigger();
    }
}

bool DistanceHeuristic::isCost()
{
    return true;
}

void DistanceHeuristic::setDebug(bool debug)
{
    this->debug = debug;
}

void DistanceHeuristic::setEndPose(geometry_msgs::Pose p)
{
    this->endPose = p;
}

void DistanceHeuristic::setTolerance2Goal(double tolerance2Goal)
{
    this->tolerance2Goal = tolerance2Goal;
}

void DistanceHeuristic::calculateHeuristic(Node *node)
{
    if(node==NULL)
        return;
    node->g_value = gCost(node);
    node->h_value = hCost(node);
    node->f_value = node->g_value + node->h_value;
}

}
