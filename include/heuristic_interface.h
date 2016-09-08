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
#ifndef HEURISTICT_H_
#define HEURISTICT_H_

#include <QString>
#include <QHash>
#include "ssppexception.h"
#include "searchspacenode.h"
#include "node.h"
#include "utils.h"
#include "geometry_msgs/Vector3.h"
using namespace std;

namespace SSPP
{

class Heuristic
{
public:
    Heuristic(){}
    virtual ~Heuristic(){}
    virtual void calculateHeuristic(Node *)=0;
    virtual bool terminateConditionReached(Node *)=0;
    virtual bool isConnectionConditionSatisfied(SearchSpaceNode*,SearchSpaceNode*)=0;
    virtual bool isFilteringConditionSatisfied(geometry_msgs::Pose pose, geometry_msgs::PoseArray& correspondingSensorPoses, double minDist, double maxDist, pcl::PointCloud<pcl::PointXYZ>& globalCloud, std::vector<pcl::PointCloud<pcl::PointXYZ> >& accuracyClusters, double accuracyThreshhold)=0;
    virtual void displayProgress(vector<Tree> tree)=0;
    virtual double pointCloudDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloudDiffPtr )=0;
    virtual void clusteringPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> >& clustersPointCloudVec, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudDiffPtr)=0;
    virtual void findClusterOuterPoints(geometry_msgs::PoseArray waypoints, pcl::PointCloud<pcl::PointXYZ>& cloudHull)=0;
    virtual void findClusterBB(pcl::PointCloud<pcl::PointXYZ> clusterPoints, geometry_msgs::Vector3 &gridSize, geometry_msgs::Pose &gridStart)=0;

    /*!
     * \brief isCost
     * \return
     * Defines whether this heuristic function is a cost or reward function
     */
    virtual bool isCost()=0;
};

}
#endif /*HEURISTICT_H_*/
