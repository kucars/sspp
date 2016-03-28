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
#include "coverage_path_planning_heuristic.h"

namespace SSPP
{

CoveragePathPlanningHeuristic::CoveragePathPlanningHeuristic(ros::NodeHandle & nh,std::string modelName, bool d)
{

    loadOBJFile(modelName.c_str(), modelPoints, triangles);
    cgalTree           = new Tree1(triangles.begin(),triangles.end());
    occlussionCulling  = new OcclusionCullingGPU(nh, modelName);
    debug              = d;
    treePub            = nh.advertise<visualization_msgs::Marker>("search_tree", 10);
}

CoveragePathPlanningHeuristic::~CoveragePathPlanningHeuristic()
{
    delete occlussionCulling;
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

bool CoveragePathPlanningHeuristic::isConnectionConditionSatisfied(SearchSpaceNode *temp, SearchSpaceNode *S)
{
    //collision check
    int intersectionsCount=0;
    //parent
    Point a(temp->location.position.x , temp->location.position.y ,temp->location.position.z );
    //child
    Point b(S->location.position.x, S->location.position.y, S->location.position.z);
    Segment seg_query(a,b);
    intersectionsCount = cgalTree->number_of_intersected_primitives(seg_query);
    if(intersectionsCount==0)
        return true;
    else
        return false;
}

void CoveragePathPlanningHeuristic::displayProgress(vector<Tree> tree)
{
    geometry_msgs::Pose child;
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint;
    for(unsigned int k=0;k<tree.size();k++)
    {
        for(int j=0;j<tree[k].children.size();j++)
        {
            child = tree[k].children[j];

            linePoint.x = tree[k].location.position.x;
            linePoint.y = tree[k].location.position.y;
            linePoint.z = tree[k].location.position.z;
            lineSegments.push_back(linePoint);
            linePoint.x = child.position.x;
            linePoint.y = child.position.y;
            linePoint.z = child.position.z;
            lineSegments.push_back(linePoint);

        }
    }
    visualization_msgs::Marker linesList = drawLines(lineSegments,1000000,2,0,0.08);
    treePub.publish(linesList);
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
    visibleCloud = occlussionCulling->extractVisibleSurface(node->senPose.p);
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
    node->coverage = occlussionCulling->calcCoveragePercent(node->cloud_filtered);
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

void CoveragePathPlanningHeuristic::loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::list<CGALTriangle>& triangles)
{

    FILE* file = fopen(filename, "rb");
    if(!file)
    {
        std::cerr << "file not exist" << std::endl;
        return;
    }

    bool has_normal = false;
    bool has_texture = false;
    char line_buffer[2000];
    while(fgets(line_buffer, 2000, file))
    {
        char* first_token = strtok(line_buffer, "\r\n\t ");
        if(!first_token || first_token[0] == '#' || first_token[0] == 0)
            continue;

        switch(first_token[0])
        {
        case 'v':
        {
            if(first_token[1] == 'n')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_normal = true;
            }
            else if(first_token[1] == 't')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_texture = true;
            }
            else
            {
                fcl::FCL_REAL x = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::FCL_REAL y = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::FCL_REAL z = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
                fcl::Vec3f p(x, y, z);
                points.push_back(p);
            }
        }
            break;
        case 'f':
        {
            CGALTriangle tri;
            char* data[30];
            int n = 0;
            while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
            {
                if(strlen(data[n]))
                    n++;
            }

            for(int t = 0; t < (n - 2); ++t)
            {
                if((!has_texture) && (!has_normal))
                {
                    Point p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Point p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Point p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
                else
                {
                    const char *v1;
                    uint indxs[3];
                    for(int i = 0; i < 3; i++)
                    {
                        // vertex ID
                        if(i == 0)
                            v1 = data[0];
                        else
                            v1 = data[t + i];

                        indxs[i] = atoi(v1) - 1;
                    }
                    Point p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Point p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Point p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
            }
        }
        }
    }
}

}
