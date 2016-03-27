/***************************************************************************
 *   Copyright (C) 2007 by Tarek Taha                                      *
 *   tataha@eng.uts.edu.au                                                 *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "pathplanner.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <QThread>

//PCL
/*
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
*/
#include <component_test/occlusion_culling_gpu.h>
#include <component_test/occlusion_culling.h>
#include "coverage_path_planning_heuristic.h"
#include "distance_heuristic.h"

using namespace SSPP;
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, float scale);
visualization_msgs::Marker drawpoints(std::vector<geometry_msgs::Point> points);

int main( int argc, char **  argv)
{
    std::cout<<"\nHere -1"; fflush(stdout);

    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh;
    ros::Publisher path_pub         = nh.advertise<visualization_msgs::Marker>("generated_path", 10);
    ros::Publisher searchSpace_pub  = nh.advertise<visualization_msgs::Marker>("search_space", 10);
    ros::Publisher connectivity_pub = nh.advertise<visualization_msgs::Marker>("connections", 10);
    ros::Publisher vector_pub       = nh.advertise<geometry_msgs::PoseArray>("pose", 10);
    ros::Publisher sen_vector_pub   = nh.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);

    std::cout<<"\nHere 0"; fflush(stdout);

    QTime timer;
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;
    gridStartPose.position.x = 0 ;
    gridStartPose.position.y = 0 ;
    gridStartPose.position.z = 0 ;
    gridSize.x = 20;
    gridSize.y = 10;
    gridSize.z = 2;

    std::string path = ros::package::getPath("sspp");
    PathPlanner * pathPlanner;
    Pose start(0.0,0.0,0,DTOR(0.0));
    Pose   end(19.0,7.0,2,DTOR(0.0));

    double robotH=0.9,robotW=0.5,narrowestPath=0.987;//is not changed
    double distanceToGoal = 1.0,regGridConRad = 1.5;

    std::cout<<"\nHere 1"; fflush(stdout);
    QPointF robotCenter(-0.3f,0.0f);    
    Robot *robot= new Robot("Robot",robotH,robotW,narrowestPath,robotCenter);

    pathPlanner = new PathPlanner(nh,robot,regGridConRad);
    std::cout<<"\nHere 2"; fflush(stdout);
    /*
    double coverageTolerance=0.5, targetCov=10;
    CoveragePathPlanningHeuristic coveragePathPlanningHeuristic(nh,"etihad_nowheels_densed.pcd",false);
    coveragePathPlanningHeuristic.setCoverageTarget(targetCov);
    coveragePathPlanningHeuristic.setCoverageTolerance(coverageTolerance);
    pathPlanner->setHeuristicFucntion(&coveragePathPlanningHeuristic);
    */

    DistanceHeuristic distanceHeuristic(nh,false);
    distanceHeuristic.setEndPose(end.p);
    distanceHeuristic.setTolerance2Goal(distanceToGoal);
    pathPlanner->setHeuristicFucntion(&distanceHeuristic);
    std::cout<<"\nHere 3"; fflush(stdout);

    pathPlanner->generateRegularGrid(gridStartPose, gridSize,1.0,false);
    //visualization (not working for some reason)
    pathPlanner->showSearchSpace();
    pathPlanner->connectNodes();
    std::cout<<"\nSpace Generation took:"<<timer.elapsed()/double(1000.00)<<" secs";
//    pathPlanner->showConnections();

    std::cout<<"\nHere 4"; fflush(stdout);
    timer.restart();
    Node * retval = pathPlanner->startSearch(start);
    std::cout<<"\nPath Finding took:"<<(timer.elapsed()/double(1000.00))<<" secs";
    std::cout<<"\nHere 5"; fflush(stdout);

    //path print and visualization
    if(retval)
    {
        pathPlanner->printNodeList();
    }
    else
    {
        std::cout<<"\nNo Path Found";
    }

    //******for visualizing the search space & connectivity********
    SearchSpaceNode *temp = pathPlanner->searchspace;
    std::vector<geometry_msgs::Point> pts;
    std::vector<geometry_msgs::Point> lineSegments1;
    while (temp != NULL)
    {
        //vertex visualization
        geometry_msgs::Point pt;
        pt.x= temp->location.position.x;
        pt.y= temp->location.position.y;
        pt.z= temp->location.position.z;
        pts.push_back(pt);
        //connectivity visualization
        for(int i=0; i < temp->children.size();i++)
        {
            geometry_msgs::Point linept;
            //point1
            linept.x = temp->location.position.x;
            linept.y = temp->location.position.y;
            linept.z = temp->location.position.z;
            lineSegments1.push_back(linept);
            //point2
            linept.x= temp->children[i]->location.position.x;
            linept.y= temp->children[i]->location.position.y;
            linept.z= temp->children[i]->location.position.z;
            lineSegments1.push_back(linept);
        }
        temp = temp->next;
    }
    visualization_msgs::Marker points_vector = drawpoints(pts);
    visualization_msgs::Marker linesList1 = drawLines(lineSegments1,3,0.03);

    //**************************************************************



    Node * p = pathPlanner->path;
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint;
    pcl::PointCloud<pcl::PointXYZ> temp_cloud, combined;
    geometry_msgs::PoseArray vec,sensor_vec;
    double dist=0;
    double yaw;

    while(p !=NULL)
    {
        tf::Quaternion qt(p->pose.p.orientation.x,p->pose.p.orientation.y,p->pose.p.orientation.z,p->pose.p.orientation.w);
        yaw = tf::getYaw(qt);
        if (p->next !=NULL)
        {
            linePoint.x = p->pose.p.position.x;
            linePoint.y = p->pose.p.position.y;
            linePoint.z = p->pose.p.position.z;
            combined += temp_cloud;
            vec.poses.push_back(p->pose.p);
            lineSegments.push_back(linePoint);
            sensor_vec.poses.push_back(p->senPose.p);
            linePoint.x = p->next->pose.p.position.x;
            linePoint.y = p->next->pose.p.position.y;
            linePoint.z = p->next->pose.p.position.z;
            lineSegments.push_back(linePoint);
            dist=dist+ Dist(p->next->pose.p,p->pose.p);
            vec.poses.push_back(p->next->pose.p);//ADD ME RANDA
            sensor_vec.poses.push_back(p->next->senPose.p);
        }
        p = p->next;
    }
    visualization_msgs::Marker linesList = drawLines(lineSegments,1,0.15);

    ros::Rate loop_rate(10);
    pathPlanner->showConnections();
    std::cout<<"\nDistance calculated from the path = "<<dist<<" \n";

    while (ros::ok())
    {

        vec.header.frame_id= "map";
        vec.header.stamp = ros::Time::now();
        vector_pub.publish(vec);

        sensor_vec.header.frame_id= "map";
        sensor_vec.header.stamp = ros::Time::now();
        sen_vector_pub.publish(sensor_vec);

        path_pub.publish(linesList);
        searchSpace_pub.publish(points_vector);
        connectivity_pub.publish(linesList1);
        ros::spinOnce();
        loop_rate.sleep();
    }
    delete robot;
    delete pathPlanner;
    return 0;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int c_color, float scale)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="/map";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = 0;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = scale;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(10000.0);
    std_msgs::ColorRGBA color;
//    color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
    if(c_color == 1)
    {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else if(c_color == 2)
    {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
    }
    else
    {
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
    }
    std::vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
   return linksMarkerMsg;
}

visualization_msgs::Marker drawpoints(std::vector<geometry_msgs::Point> points)
{
    visualization_msgs::Marker pointMarkerMsg;
    pointMarkerMsg.header.frame_id="/map";
    pointMarkerMsg.header.stamp=ros::Time::now();
    pointMarkerMsg.ns="point_marker";
    pointMarkerMsg.id = 2000;
    pointMarkerMsg.type = visualization_msgs::Marker::POINTS;
    pointMarkerMsg.scale.x = 0.1;
    pointMarkerMsg.scale.y = 0.1;
    pointMarkerMsg.action  = visualization_msgs::Marker::ADD;
    pointMarkerMsg.lifetime  = ros::Duration(100.0);
    std_msgs::ColorRGBA color;
    color.r = 0.0f; color.g=1.0f; color.b=0.0f, color.a=1.0f;
    std::vector<geometry_msgs::Point>::iterator pointsIterator;
    for(pointsIterator = points.begin();pointsIterator != points.end();pointsIterator++)
    {
        pointMarkerMsg.points.push_back(*pointsIterator);
        pointMarkerMsg.colors.push_back(color);
    }
   return pointMarkerMsg;
}
