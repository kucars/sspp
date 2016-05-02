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

CoveragePathPlanningHeuristic::CoveragePathPlanningHeuristic(ros::NodeHandle & nh, std::string collisionCheckModelP, std::string occlusionCullingModelN, bool d, bool gradualV, int hType)
{

    loadOBJFile(collisionCheckModelP.c_str(), modelPoints, triangles);
    cgalTree             = new Tree1(triangles.begin(),triangles.end());
    occlussionCulling    = new OcclusionCullingGPU(nh, occlusionCullingModelN);
    meshSurface          = new MeshSurface(nh);
    debug                = d;
    gradualVisualization = gradualV;
    heuristicType        = hType;
    treePub              = nh.advertise<visualization_msgs::Marker>("search_tree", 10);
    coveredPointsPub     = nh.advertise<sensor_msgs::PointCloud2>("gradual_coverage", 100);;
    pathPointPub         = nh.advertise<visualization_msgs::Marker>("path_point" , 10);
    pathPub              = nh.advertise<visualization_msgs::Marker>("path_testing", 10);
    accuracySum          = 0.0;
    extraCovSum          = 0.0;
    extraAreaSum         = 0.0;

    Triangles aircraftCGALT ;
    meshSurface->loadOBJFile(collisionCheckModelP.c_str(), modelPoints, aircraftCGALT);
    aircraftArea = meshSurface->calcCGALMeshSurfaceArea(aircraftCGALT);
}

CoveragePathPlanningHeuristic::~CoveragePathPlanningHeuristic()
{
    delete occlussionCulling;
}

bool CoveragePathPlanningHeuristic::terminateConditionReached(Node *node)
{
    double deltaCoverage;
    deltaCoverage = coverageTarget - node->coverage;
    std::cout<<"\n\n ****************** Total Coverage %: "<<node->coverage<<" **************************"<<std::endl;

    if (debug)
        std::cout<<"Delta Coverage:"<<deltaCoverage<<"\n";

    if ( deltaCoverage <= coverageTolerance)
    {
        std::cout<<"\n\nAverage Accuracy per viewpoint is "<<accuracySum/accuracyPerViewpointAvg.size()<<std::endl;
        std::cout<<"Average extra coverage per viewpoint is "<<extraCovSum/extraCovPerViewpointAvg.size()<<std::endl;
        std::cout<<"Average extra Area per viewpoint is "<<extraAreaSum/extraAreaperViewpointAvg.size()<<"\n\n"<<std::endl;

        return true;
    }
    else
    {
        if(gradualVisualization)
            displayGradualProgress(node);
        return false;
    }
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
    visualization_msgs::Marker linesList = drawLines(lineSegments,1,2,1000000,0.08);
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

    for(int i=0; i<node->senPoses.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ> temp;
        temp = occlussionCulling->extractVisibleSurface(node->senPoses[i].p);
        visibleCloud += temp;

    }
//    visibleCloud = occlussionCulling->extractVisibleSurface(node->senPose.p);

    if(node->parent)
    {

        collectiveCloud.points = node->parent->cloud_filtered->points;
        collectiveCloud +=visibleCloud;
        tempCloud->points = collectiveCloud.points;

        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setInputCloud (tempCloud);

        voxelgrid.setLeafSize (0.5f, 0.5f, 0.5f);
        voxelgrid.filter(*node->cloud_filtered);

        node->coverage = occlussionCulling->calcCoveragePercent(node->cloud_filtered);

        node->h_value  = 0;
        node->g_value  = 0;

        double f=0,d,c,a;

        // Using the coverage percentage
        d = Dist(node->pose.p,node->parent->pose.p);
        c = node->coverage - node->parent->coverage;
        extraCovPerViewpointAvg.push_back(c);
        extraCovSum +=c;
        if(debug)
        {
            std::cout<<"\nchild collective cloud after filtering size: "<<node->cloud_filtered->size()<<"\n";
            std::cout<<"parent distance :"<<node->parent->distance<<" current node distance: "<<node->distance<<"\n";
            std::cout<<"parent coverage :"<<node->parent->coverage<<" current node coverage: "<<node->coverage<<"\n";
            std::cout<<"Calculated local distance d:"<<d<<" comulative distance: "<<node->distance<<"\n";
            std::cout<<"extra coverage c : "<<c<<"\n";
        }


        if(d!=0.0){
            if(heuristicType==SurfaceCoverageH || heuristicType==SurfaceCoveragewithOrientationH)
                f = node->parent->f_value + ((1/d)*c);
            else if(heuristicType==SurfaceCoveragewithAccuracyH)
            {
                double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
                a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;
                accuracyPerViewpointAvg.push_back(a);
                accuracySum += avgAcc;
                f = node->parent->f_value + ((1/d)*c*a);
            }else if(heuristicType==SurfaceAreaCoverageH)
            {
                Triangles tempTri;
                meshSurface->meshingPCL(visibleCloud, tempTri, false);
                meshSurface->setCGALMeshA(node->parent->surfaceTriangles);
                meshSurface->setCGALMeshB(tempTri);
                node->surfaceTriangles= node->parent->surfaceTriangles;

                double extraCovArea = meshSurface->getExtraArea(node->surfaceTriangles);//here the function should increase the number of surfaceTriangles adding (extra triangles)
                if(debug)
                {
                    std::cout<<"triangles :"<<tempTri.size()<<std::endl;
                    std::cout<<"after getting extra Tri : "<<node->surfaceTriangles.size()<<std::endl<<std::endl;
                }

                node->coverage = (meshSurface->calcCGALMeshSurfaceArea(node->surfaceTriangles)/aircraftArea )* 100; //accumelated coverage % instead of the accumelated coverage in terms of the points

                extraAreaperViewpointAvg.push_back(extraCovArea);
                double AreaCoveragePercent = (extraCovArea/aircraftArea)*100;// extra surface area % used instead of c
                extraAreaSum += AreaCoveragePercent;

                if(debug)
                {
                    Triangles temp;
                    double interCovArea = meshSurface->getIntersectionArea(temp);
                    std::cout<<"node viewpoint area: "<<meshSurface->calcCGALMeshSurfaceArea(tempTri) <<"intersection B:"<<interCovArea<<std::endl;
                    std::cout<<"Aircraft Area: "<<aircraftArea <<"extra area: "<<extraCovArea<<" extra area percent: "<<AreaCoveragePercent<<std::endl;
                }

                f = node->parent->f_value + (1/d)*AreaCoveragePercent;
            }
        }
        else{
            if(heuristicType==SurfaceCoverageH)
                f = node->parent->f_value + c;
            else if(heuristicType==SurfaceCoveragewithOrientationH) {
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
                normAngle=1-angle/(2*M_PI);
                f = node->parent->f_value + normAngle*c;
            } else if(heuristicType==SurfaceCoveragewithAccuracyH){
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
                normAngle=1-angle/(2*M_PI);
                double avgAcc = occlussionCulling->calcAvgAccuracy(visibleCloud);
                accuracySum += avgAcc;
                a = (occlussionCulling->maxAccuracyError - occlussionCulling->calcAvgAccuracy(visibleCloud))/occlussionCulling->maxAccuracyError;
                f = node->parent->f_value + a*c*normAngle;
            }else if(heuristicType==SurfaceAreaCoverageH)
            {
                tf::Quaternion qtParent(node->parent->pose.p.orientation.x,node->parent->pose.p.orientation.y,node->parent->pose.p.orientation.z,node->parent->pose.p.orientation.w);
                tf::Quaternion qtNode(node->pose.p.orientation.x,node->pose.p.orientation.y,node->pose.p.orientation.z,node->pose.p.orientation.w);
                double angle, normAngle;
                angle=qtParent.angleShortestPath(qtNode);
                normAngle=1-angle/(2*M_PI);

                Triangles tempTri;
                meshSurface->meshingPCL(visibleCloud, tempTri,false);
                meshSurface->setCGALMeshA(node->parent->surfaceTriangles);
                meshSurface->setCGALMeshB(tempTri);
                node->surfaceTriangles= node->parent->surfaceTriangles;
                double extraCovArea = meshSurface->getExtraArea(node->surfaceTriangles);//here the function should increase the number of surfaceTriangles adding (extra triangles)

                if(debug)
                {
                    std::cout<<"triangles :"<<tempTri.size()<<std::endl;
                    std::cout<<"after getting extra Tri : "<<node->surfaceTriangles.size()<<std::endl<<std::endl;
                }

                node->coverage = (meshSurface->calcCGALMeshSurfaceArea(node->surfaceTriangles)/aircraftArea )* 100; //accumelated coverage % instead of the accumelated coverage in terms of the points

                extraAreaperViewpointAvg.push_back(extraCovArea);
                double AreaCoveragePercent = (extraCovArea/aircraftArea)*100; // extra surface area % used instead of c
                extraAreaSum += AreaCoveragePercent;

                if(debug)
                {
                    Triangles temp;
                    double interCovArea = meshSurface->getIntersectionArea(temp);
                    std::cout<<"node viewpoint area: "<<meshSurface->calcCGALMeshSurfaceArea(tempTri) <<"intersection B:"<<interCovArea<<std::endl;
                    std::cout<<"Aircraft Area: "<<aircraftArea <<"extra area: "<<extraCovArea<<" extra area percent: "<<AreaCoveragePercent<<std::endl;
                }
                f = node->parent->f_value + AreaCoveragePercent*normAngle;
            }
        }
        node->f_value  = f;
        node->distance = node->parent->distance + d;
        if(debug)
            std::cout<<"parent f value calculation: "<<f<<"\n";
    }else
        node->f_value =0;//root node

    std::cout<<"finished calculation"<<std::endl;
}
void CoveragePathPlanningHeuristic::displayGradualProgress(Node *node)
{
            //########display the covered points##########
            sensor_msgs::PointCloud2 cloud1;
            pcl::toROSMsg(*(node->cloud_filtered), cloud1);
            cloud1.header.frame_id = "map";
            cloud1.header.stamp = ros::Time::now();
            coveredPointsPub.publish(cloud1);


            //########display the point selected##########
    //        std::vector<geometry_msgs::Point> points;
    //        geometry_msgs::Point linept;
    //        linept.x = node->pose.p.position.x; linept.y = node->pose.p.position.y; linept.z = node->pose.p.position.z;
    //        points.push_back(linept);
    //        visualization_msgs::Marker pointsList = drawPoints(points,10,1,10000,0.05);
    //        pathPointPub.publish(pointsList);


            int coveI = (int)node->coverage;

            //########display FOV##########
    //        if (coveI != 0 && %10==0)
    //            occlusionCulling->visualizeFOV(node->senPose.p);

            //########display the path every 1% coverage########
            if (debug == true)
                std::cout<<"\n\n\n\n**********************COVERAGE delta:" <<coveI<<"\n\n\n\n";
            if ( coveI%1==0)
            {
                if (debug == true)
                    std::cout<<"INSIDE PUBLISHING"<<"\n";


                //  publish path and pring the path each 1%
                ofstream path_file;
                std::string path = ros::package::getPath("sspp");
                std::string fileloc = path+ "/resources/path_testfile.txt";
                path_file.open(fileloc.c_str());
                std::vector<geometry_msgs::Point> lines;
                geometry_msgs::Point linepoint;
                Node *test_path;
                test_path = node;
                double yaw;
                while (test_path != NULL)
                {
                    tf::Quaternion qt(test_path->pose.p.orientation.x,test_path->pose.p.orientation.y,test_path->pose.p.orientation.z,test_path->pose.p.orientation.w);
                    yaw = tf::getYaw(qt);
                    path_file << test_path->pose.p.position.x<<" "<<test_path->pose.p.position.y<<" "<<test_path->pose.p.position.z<<" "<<yaw<<"\n";
                    if (test_path->parent != NULL)
                    {
                        linepoint.x = test_path->pose.p.position.x; linepoint.y = test_path->pose.p.position.y; linepoint.z = test_path->pose.p.position.z;
                        lines.push_back(linepoint);
                        linepoint.x = test_path->parent->pose.p.position.x; linepoint.y = test_path->parent->pose.p.position.y; linepoint.z = test_path->parent->pose.p.position.z;
                        lines.push_back(linepoint);
                    }
                    test_path = test_path->parent;
                }

                path_file.close();
                visualization_msgs::Marker linesList = drawLines(lines,333333,1,1000000,0.2);
                pathPub.publish(linesList);
            }
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
