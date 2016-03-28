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

#include "pathplanner.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

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
#include "rviz_drawing_tools.h"

using namespace SSPP;

//TODO: use rviz_visual_tools for drawing stuff

int main( int argc, char **  argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh;
    ros::Publisher pathPub         = nh.advertise<visualization_msgs::Marker>("generated_path", 10);
    ros::Publisher searchSpacePub  = nh.advertise<visualization_msgs::Marker>("search_space", 10);
    ros::Publisher connectivityPub = nh.advertise<visualization_msgs::Marker>("connections", 10);
    ros::Publisher robotPosePub    = nh.advertise<geometry_msgs::PoseArray>("robot_pose", 10);
    ros::Publisher sensorPosePub   = nh.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);

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

    PathPlanner * pathPlanner;
    Pose start(0.0,0.0,0,DTOR(0.0));
    Pose   end(19.0,7.0,2,DTOR(0.0));

    double robotH=0.9,robotW=0.5,narrowestPath=0.987;//is not changed
    double distanceToGoal = 1.0,regGridConRad = 1.5;

    std::cout<<"\nHere 1"; fflush(stdout);
    QPointF robotCenter(-0.3f,0.0f);    
    Robot *robot= new Robot("Robot",robotH,robotW,narrowestPath,robotCenter);
    // Every how many iterations to display the tree
    int progressDisplayFrequency = 10;
    pathPlanner = new PathPlanner(nh,robot,regGridConRad,progressDisplayFrequency);
    std::cout<<"\nHere 2"; fflush(stdout);
    /*
    double coverageTolerance=0.5, targetCov=10;
    std::string modelPath = ros::package::getPath("component_test") + "/src/mesh/etihad_nowheels.obj";
    CoveragePathPlanningHeuristic coveragePathPlanningHeuristic(nh,modelPath,false);
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
    std::vector<geometry_msgs::Point> pts = pathPlanner->getSearchSpace();
    searchSpacePub.publish(drawPoints(pts,3,100));
    std::cout<<"\n"<<QString("\n---->>> Total Nodes in search Space =%1").arg(pts.size()).toStdString();

    pathPlanner->connectNodes();
    std::cout<<"\nSpace Generation took:"<<timer.elapsed()/double(1000.00)<<" secs";


    std::vector<geometry_msgs::Point> lineSegments;
    visualization_msgs::Marker linesList;

    lineSegments = pathPlanner->getConnections();
    linesList = drawLines(lineSegments,2000000,3,100000000,0.08);
    connectivityPub.publish(linesList);

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
    visualization_msgs::Marker searchSpaceConnectivityList;
    lineSegments.clear();
    pts.clear();
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
            lineSegments.push_back(linept);
            //point2
            linept.x= temp->children[i]->location.position.x;
            linept.y= temp->children[i]->location.position.y;
            linept.z= temp->children[i]->location.position.z;
            lineSegments.push_back(linept);
        }
        temp = temp->next;
    }
    visualization_msgs::Marker searchSpacePoints = drawPoints(pts,1,0);
    //Link,id, color,duration,scale
    searchSpaceConnectivityList = drawLines(lineSegments,1,3,0,0.03);

    Node * path = pathPlanner->path;
    geometry_msgs::Point linePoint;
    pcl::PointCloud<pcl::PointXYZ> temp_cloud, combined;
    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose,sensorPose;
    visualization_msgs::Marker pathList;
    double dist=0;
    double yaw;
    while(path !=NULL)
    {
        tf::Quaternion qt(path->pose.p.orientation.x,path->pose.p.orientation.y,path->pose.p.orientation.z,path->pose.p.orientation.w);
        yaw = tf::getYaw(qt);
        if (path->next !=NULL)
        {
            linePoint.x = path->pose.p.position.x;
            linePoint.y = path->pose.p.position.y;
            linePoint.z = path->pose.p.position.z;
            combined += temp_cloud;
            robotPose.poses.push_back(path->pose.p);
            pathSegments.push_back(linePoint);
            sensorPose.poses.push_back(path->senPose.p);
            linePoint.x = path->next->pose.p.position.x;
            linePoint.y = path->next->pose.p.position.y;
            linePoint.z = path->next->pose.p.position.z;
            pathSegments.push_back(linePoint);
            dist=dist+ Dist(path->next->pose.p,path->pose.p);
            robotPose.poses.push_back(path->next->pose.p);
            sensorPose.poses.push_back(path->next->senPose.p);
        }
        path = path->next;
    }
    //Link,id, color,duration,scale
   pathList = drawLines(pathSegments,2,1,0,0.15);

    ros::Rate loopRate(10);
    std::cout<<"\nDistance calculated from the path = "<<dist<<" \n";

    while (ros::ok())
    {

        robotPose.header.frame_id= "map";
        robotPose.header.stamp = ros::Time::now();
        robotPosePub.publish(robotPose);

        sensorPose.header.frame_id= "map";
        sensorPose.header.stamp = ros::Time::now();
        sensorPosePub.publish(sensorPose);

        pathPub.publish(pathList);
        searchSpacePub.publish(searchSpacePoints);
        connectivityPub.publish(searchSpaceConnectivityList);

        ros::spinOnce();
        loopRate.sleep();
    }
    delete robot;
    delete pathPlanner;
    return 0;
}


