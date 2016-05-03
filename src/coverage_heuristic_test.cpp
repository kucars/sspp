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

#include <component_test/occlusion_culling_gpu.h>
#include <component_test/occlusion_culling.h>
#include "coverage_path_planning_heuristic.h"
#include "distance_heuristic.h"
#include "rviz_drawing_tools.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
//#include <component_test/mesh_surface.h>
using namespace SSPP;

int main( int argc, char **  argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh;

    ros::Publisher originalCloudPub  = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher visiblePub        = nh.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    ros::Publisher pathPub           = nh.advertise<visualization_msgs::Marker>("generated_path", 10);
    ros::Publisher searchSpacePub    = nh.advertise<visualization_msgs::Marker>("search_space", 10);
    ros::Publisher connectionsPub    = nh.advertise<visualization_msgs::Marker>("connections", 10);
    ros::Publisher robotPosePub      = nh.advertise<geometry_msgs::PoseArray>("robot_pose", 10);
    ros::Publisher sensorPosePub     = nh.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);
    ros::Publisher robotPoseSSPub      = nh.advertise<geometry_msgs::PoseArray>("SS_robot_pose", 10);
    ros::Publisher sensorPoseSSPub     = nh.advertise<geometry_msgs::PoseArray>("SS_sensor_pose", 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (ros::package::getPath("component_test")+"/src/pcd/etihad_nowheels_nointernal_newdensed.pcd", *originalCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr coveredCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    OcclusionCullingGPU occlusionCulling(nh,"etihad_nowheels_nointernal_newdensed.pcd");

    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("map","/sspp_visualisation"));
    visualTools->deleteAllMarkers();
    visualTools->setLifetime(0.2);

    QTime timer;
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;
    gridStartPose.position.x = -36 ;
    gridStartPose.position.y = -45 ;
    gridStartPose.position.z = 0 ;
    gridSize.x = 72;
    gridSize.y = 90;
    gridSize.z = 21;

    PathPlanner * pathPlanner;
    Pose start(3.0,-34.0,9,DTOR(0.0));
    Pose   end(19.0,7.0,2,DTOR(0.0));

    double robotH=0.9,robotW=0.5,narrowestPath=0.987;//is not changed
    double distanceToGoal = 1.0,regGridConRad = 2.5;

    QPointF robotCenter(-0.3f,0.0f);
    Robot *robot= new Robot("Robot",robotH,robotW,narrowestPath,robotCenter);
    Sensors sensor1(58,45,0.255,0.7,6.0,640,480,Vec3f(0,0,-0.055), Vec3f(0,0.093,0));
    Sensors sensor2(58,45,0.255,0.7,6.0,640,480,Vec3f(0,0,0.055), Vec3f(0,0.0,0));

    std::vector<Sensors> sensors;
    sensors.push_back(sensor1);
//    sensors.push_back(sensor2);

    // Every how many iterations to display the tree
    int progressDisplayFrequency = 1;
    pathPlanner = new PathPlanner(nh,robot,regGridConRad,progressDisplayFrequency,sensors);
    // This causes the planner to pause for the desired amount of time and display the search tree, useful for debugging
    pathPlanner->setDebugDelay(0.0);



    double coverageTolerance=0.5, targetCov=5;
    std::string collisionCheckModelPath = ros::package::getPath("component_test") + "/src/mesh/etihad_nowheels_nointernal_new.obj";
    std::string occlusionCullingModelName = "etihad_nowheels_nointernal_newdensed.pcd";
    CoveragePathPlanningHeuristic coveragePathPlanningHeuristic(nh,collisionCheckModelPath,occlusionCullingModelName,false, true, SurfaceCoverageH);
    coveragePathPlanningHeuristic.setCoverageTarget(targetCov);
    coveragePathPlanningHeuristic.setCoverageTolerance(coverageTolerance);
    pathPlanner->setHeuristicFucntion(&coveragePathPlanningHeuristic);

    std::string str1 = ros::package::getPath("sspp")+"/resources/SearchSpaceUAV_1.5m_1to4_NEW_etihadNoWheelsNOInternal.txt";
    std::string str2 = ros::package::getPath("sspp")+"/resources/SearchSpaceCam_1.5m_1to4_NEW_etihadNoWheelsNOInternal.txt";
    const char * filename1 = str1.c_str();
    const char * filename2 = str2.c_str();
//    pathPlanner->loadRegularGrid(filename1,filename2);
    /*
    DistanceHeuristic distanceHeuristic(nh,false);
    distanceHeuristic.setEndPose(end.p);
    distanceHeuristic.setTolerance2Goal(distanceToGoal);
    pathPlanner->setHeuristicFucntion(&distanceHeuristic);
    */
    // Generate Grid Samples and visualise it
    pathPlanner->generateRegularGrid(gridStartPose, gridSize,1.5,true,180,true);
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
    std::cout<<"\n"<<QString("\n---->>> Total Nodes in search Space =%1").arg(searchSpaceNodes.size()).toStdString();
    geometry_msgs::PoseArray robotPoseSS,sensorPoseSS;
    pathPlanner->getRobotSensorPoses(robotPoseSS,sensorPoseSS);
    visualization_msgs::Marker searchSpaceMarker = drawPoints(searchSpaceNodes,2,1000000);
//    visualTools->publishSpheres(searchSpaceNodes,rviz_visual_tools::PURPLE,0.1,"search_space_nodes");

    // Connect nodes and visualise it
    pathPlanner->connectNodes();
    std::cout<<"\nSpace Generation took:"<<timer.elapsed()/double(1000.00)<<" secs";
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
    visualization_msgs::Marker connectionsMarker = drawLines(searchSpaceConnections,10000,3,100000000,0.03);

//    visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE,"search_space");

    // Find path and visualise it
    timer.restart();
    Node * retval = pathPlanner->startSearch(start);
    std::cout<<"\nPath Finding took:"<<(timer.elapsed()/double(1000.00))<<" secs";

    //path print and visualization
    if(retval)
    {
        pathPlanner->printNodeList();
    }
    else
    {
        std::cout<<"\nNo Path Found";
    }
    std::cout<<"\nPath Finding took:"<<(timer.elapsed()/double(1000.00))<<" secs";

    Node * path = pathPlanner->path;
    geometry_msgs::Point linePoint;
    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose,sensorPose;
    double dist=0;
    double yaw;
    pcl::PointCloud<pcl::PointXYZ> temp_cloud, combined;

    //write to file
    ofstream pathFile;
    std::stringstream ss,cc;
    ss << targetCov;
    cc <<regGridConRad;
    std::string file_loc = ros::package::getPath("sspp")+"/resources/"+cc.str()+"_"+ss.str()+"%path_newtests1to4_"+occlusionCullingModelName+"O.txt";
    pathFile.open (file_loc.c_str());
    while(path !=NULL)
    {
        tf::Quaternion qt(path->pose.p.orientation.x,path->pose.p.orientation.y,path->pose.p.orientation.z,path->pose.p.orientation.w);
        yaw = tf::getYaw(qt);
        pathFile << path->pose.p.position.x<<" "<<path->pose.p.position.y<<" "<<path->pose.p.position.z<<" "<<yaw<<"\n";
        if (path->next !=NULL)
        {
            linePoint.x = path->pose.p.position.x;
            linePoint.y = path->pose.p.position.y;
            linePoint.z = path->pose.p.position.z;
            robotPose.poses.push_back(path->pose.p);
            for(int i =0; i<path->senPoses.size();i++)
            {
                sensorPose.poses.push_back(path->senPoses[i].p);
                temp_cloud=occlusionCulling.extractVisibleSurface(path->senPoses[i].p);
                combined += temp_cloud;
            }
            pathSegments.push_back(linePoint);


            linePoint.x = path->next->pose.p.position.x;
            linePoint.y = path->next->pose.p.position.y;
            linePoint.z = path->next->pose.p.position.z;
            robotPose.poses.push_back(path->next->pose.p);
            for(int i =0; i<path->next->senPoses.size();i++)
            {
                sensorPose.poses.push_back(path->next->senPoses[i].p);
                temp_cloud=occlusionCulling.extractVisibleSurface(path->next->senPoses[i].p);
                combined += temp_cloud;
            }
            pathSegments.push_back(linePoint);

            dist=dist+ Dist(path->next->pose.p,path->pose.p);
        }
        path = path->next;
    }
    pathFile.close();
//    visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"generated_path");
    visualization_msgs::Marker pathMarker = drawLines(pathSegments,20000,1,10000000,0.08);
    coveredCloudPtr->points=combined.points;

    ros::Rate loopRate(10);
    std::cout<<"\nDistance calculated from the path: "<<dist<<"m\n";
    std::cout<<"Covered Cloud % : "<<occlusionCulling.calcCoveragePercent(coveredCloudPtr)<<"%\n";

    sensor_msgs::PointCloud2 cloud1,cloud2;
    while (ros::ok())
    {
        /*
        visualTools->resetMarkerCounts();
        for(int i=0;i<robotPose.poses.size();i++)
        {
            visualTools->publishArrow(robotPose.poses[i],rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE,0.3);
        }
        visualTools->publishSpheres(searchSpaceNodes,rviz_visual_tools::PURPLE,0.1,"search_space_nodes");
        visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE,"search_space");
        visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"generated_path");
        */

        pcl::toROSMsg(*originalCloudPtr, cloud1); //cloud of original (white) using original cloud
        cloud1.header.stamp = ros::Time::now();
        cloud1.header.frame_id = "map"; //change according to the global frame please!!
        originalCloudPub.publish(cloud1);

        pcl::toROSMsg(*coveredCloudPtr, cloud2); //cloud of original (white) using original cloud
        cloud2.header.stamp = ros::Time::now();
        cloud2.header.frame_id = "map"; //change according to the global frame please!!
        visiblePub.publish(cloud2);

        searchSpacePub.publish(searchSpaceMarker);
        connectionsPub.publish(connectionsMarker);
        pathPub.publish(pathMarker);

        robotPose.header.frame_id= "map";
        robotPose.header.stamp = ros::Time::now();
        robotPosePub.publish(robotPose);

        sensorPose.header.frame_id= "map";
        sensorPose.header.stamp = ros::Time::now();
        sensorPosePub.publish(sensorPose);

        robotPoseSS.header.frame_id= "map";
        robotPoseSS.header.stamp = ros::Time::now();
        robotPoseSSPub.publish(robotPoseSS);

        sensorPoseSS.header.frame_id= "map";
        sensorPoseSS.header.stamp = ros::Time::now();
        sensorPoseSSPub.publish(sensorPoseSS);

        ros::spinOnce();
        loopRate.sleep();
    }
    delete robot;
    delete pathPlanner;
    return 0;
}


