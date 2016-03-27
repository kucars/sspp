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
#include "coverage_path_planning_heuristic.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace SSPP
{

CoveragePathPlanningHeuristic::CoveragePathPlanningHeuristic(ros::NodeHandle & nh,std::string modelName, bool d)
{
    obj   = new OcclusionCullingGPU(nh, modelName);
    debug = d;
}

CoveragePathPlanningHeuristic::~CoveragePathPlanningHeuristic()
{
    delete obj;
}

bool CoveragePathPlanningHeuristic::terminateConditionReached(Node *node)
{
    double deltaCoverage;
    deltaCoverage = coverageTarget - node->coverage;

    if (debug)
        std::cout<<"Delta Coverage:"<<deltaCoverage<<"\n";

    if ( deltaCoverage <= coverageTolerance)
        return true;
    else
        return false;
}

bool CoveragePathPlanningHeuristic::isCost()
{
    return false;
}

void CoveragePathPlanningHeuristic::setCoverageTarget(double coverageTarget)
{
    this->coverageTarget = coverageTarget;
}

void CoveragePathPlanningHeuristic::setCoverageTolerance(double coverageTolerance)
{
    this->coverageTolerance = coverageTolerance;
}

void CoveragePathPlanningHeuristic::setDebug(bool debug)
{
    this->debug = debug;
}

void CoveragePathPlanningHeuristic::calculateHeuristic(Node *node)
{
    if(node==NULL)
        return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> visibleCloud, collectiveCloud;
    visibleCloud = obj->extractVisibleSurface(node->senPose.p);
    if(node->parent)
    {
        collectiveCloud.points = node->parent->cloud_filtered->points;
    }
    collectiveCloud +=visibleCloud;
    tempCloud->points = collectiveCloud.points;

    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud (tempCloud);
    voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
    voxelgrid.filter(*node->cloud_filtered);
    node->coverage = obj->calcCoveragePercent(node->cloud_filtered);
    node->h_value  = 0;
    node->g_value  = 0;

    double f=0,d,c;

    // Using the coverage percentage
    d = Dist(node->pose.p,node->parent->pose.p);
    c = node->coverage - node->parent->coverage;

    if(debug)
    {
        std::cout<<"\nchild collective cloud after filtering size: "<<node->cloud_filtered->size()<<"\n";
        std::cout<<"parent distance :"<<node->parent->distance<<" current node distance: "<<node->distance<<"\n";
        std::cout<<"parent coverage :"<<node->parent->coverage<<" current node coverage: "<<node->coverage<<"\n";
        std::cout<<"Calculated local distance d:"<<d<<" comulative distance: "<<node->distance<<"\n";
        std::cout<<"extra coverage c : "<<c<<"\n";
    }

    if(d!=0.0)
        f = node->parent->f_value + ((1/d)*c);
    else
        f = node->parent->f_value + c;

    node->f_value  = f;
    node->distance = node->parent->distance + d;
    if(debug)
        std::cout<<"parent f value calculation: "<<f<<"\n";
}

}
