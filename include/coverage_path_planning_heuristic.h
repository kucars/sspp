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
#ifndef CPP_HEURISTICT_H_
#define CPP_HEURISTICT_H_
#include <stdio.h>
#include <stdlib.h>
#include <component_test/occlusion_culling_gpu.h>
#include <component_test/occlusion_culling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Segment_3.h>
#include <CGAL/AABB_triangle_primitive.h>
#include "fcl/math/vec_3f.h"
#include "fcl/math/transform.h"
#include "ssppexception.h"
#include "node.h"
#include "heuristic_interface.h"
#include "rviz_drawing_tools.h"
#include "component_test/mesh_surface.h"
using namespace std;

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line1;
typedef K::Point_3 Point;
typedef K::Segment_3 Segment;
typedef K::Triangle_3 CGALTriangle;
typedef std::list<CGALTriangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree1;
typedef CGAL::Cartesian_converter<K,exactKernel > SimpleToExactConverter;

enum{SurfaceCoverageH,SurfaceCoveragewithOrientationH,SurfaceCoveragewithAccuracyH,SurfaceAreaCoverageH};

namespace SSPP
{

class CoveragePathPlanningHeuristic:public Heuristic
{
public:
    CoveragePathPlanningHeuristic(ros::NodeHandle & nh, std::string collisionCheckModelP, std::string occlusionCullingModelN, bool d,bool gradualV, int hType);
    ~CoveragePathPlanningHeuristic();
    bool isCost();
    void setCoverageTarget(double coverageTarget);
    void setCoverageTolerance(double coverageTolerance);
    void setDebug(bool debug);
    void calculateHeuristic(Node *n);
    bool terminateConditionReached(Node *node);
    bool isConnectionConditionSatisfied(SearchSpaceNode *temp, SearchSpaceNode *S);
    bool isFilteringConditionSatisfied(geometry_msgs::Pose pose, geometry_msgs::PoseArray &correspondingSensorPoses, double minDist, double maxDist);
    void displayProgress(vector<Tree> tree);
    void displayGradualProgress(Node *node);

private:
    void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::list<CGALTriangle>& triangles);
    OcclusionCullingGPU* occlussionCulling;
    MeshSurface* meshSurface;
    bool debug;
    bool gradualVisualization;
    int heuristicType;
    double coverageTarget;
    double coverageTolerance;
    std::vector<double> accuracyPerViewpointAvg, extraCovPerViewpointAvg, extraAreaperViewpointAvg;
    double accuracySum, extraCovSum, extraAreaSum, aircraftArea;
    ros::Publisher treePub;
    ros::Publisher coveredPointsPub;
    ros::Publisher pathPointPub;
    ros::Publisher pathPub;
    std::list<CGALTriangle> triangles;
    std::vector<fcl::Vec3f> modelPoints;
    Tree1* cgalTree;
};

}
#endif /*CPP_HEURISTICT_H_*/
