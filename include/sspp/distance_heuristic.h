/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
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
#ifndef DIST_HEURISTICT_H_
#define DIST_HEURISTICT_H_

#include "ssppexception.h"
#include "node.h"
#include "heuristic_interface.h"
#include "rviz_drawing_tools.h"
#include <octomap_world/octomap_manager.h>
#include "rviz_visual_tools/rviz_visual_tools.h"

namespace SSPP
{

class DistanceHeuristic:public Heuristic
{
public:
    DistanceHeuristic(ros::NodeHandle &n, bool d=false,volumetric_mapping::OctomapManager * manager=NULL, rviz_visual_tools::RvizVisualToolsPtr visualTools_ = NULL);
    ~DistanceHeuristic(){}
    double gCost(Node *node);
    double hCost(Node *node);
    bool isCost();
    void setDebug(bool debug);
    void setEndPose(geometry_msgs::Pose pose);
    void setTolerance2Goal(double tolerance2Distance);
    void calculateHeuristic(Node *n);
    bool terminateConditionReached(Node *node);
    bool isConnectionConditionSatisfied(SearchSpaceNode*temp, SearchSpaceNode*S);
    bool isConnectionConditionSatisfied(geometry_msgs::Pose temp, geometry_msgs::Pose S);
    bool isFilteringConditionSatisfied(geometry_msgs::Pose pose, geometry_msgs::PoseArray &correspondingSensorPoses, double minDist, double maxDist, pcl::PointCloud<pcl::PointXYZ>& globalCloud, std::vector<pcl::PointCloud<pcl::PointXYZ> >& accuracyClusters, double accuracyThreshhold);
    double pointCloudDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudDiffPtr );
    void clusteringPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> >& clustersPointCloudVec, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudDiffPtr);
    void findClusterOuterPoints(geometry_msgs::PoseArray waypoints, pcl::PointCloud<pcl::PointXYZ>& cloudHull);
    void findClusterBB(pcl::PointCloud<pcl::PointXYZ> clusterPoints, geometry_msgs::Vector3 &gridSize, geometry_msgs::Pose &gridStart);
    void displayProgress(vector<Tree> tree);

private:
    bool debug;
    geometry_msgs::Pose endPose;
    double tolerance2Goal;
    ros::Publisher treePub;
    ros::Publisher pathPointPub;
    ros::Publisher pathPub;
    std::vector<geometry_msgs::Point> points;
    std::vector<geometry_msgs::Point> lines;
    geometry_msgs::Point linepoint;
    volumetric_mapping::OctomapManager * manager;
    rviz_visual_tools::RvizVisualToolsPtr visualTools;
};

}
#endif /*DIST_HEURISTICT_H_*/
