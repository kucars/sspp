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
#include "heuristic_interface.h"
#include "distance_heuristic.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace SSPP
{

DistanceHeuristic::DistanceHeuristic(ros::NodeHandle & nh, bool d)
{
    debug = d;
}

double DistanceHeuristic::gCost(Node *node)
{
    double cost;
    if(node == NULL || node->parent==NULL)
        return 0.0;
    cost = node->parent->g_value + Dist(node->pose.p,node->parent->pose.p);
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
    if ( deltaDist <= tolerance2Goal)
        return true;
    else
        return false;
}

bool DistanceHeuristic::isCost()
{
    return false;
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
