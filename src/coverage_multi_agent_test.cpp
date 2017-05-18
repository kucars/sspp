/***************************************************************************
 *   Copyright (C) 2006 - 2017 by                                          *
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

#include "sspp/pathplanner.h"
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
#include "sspp/coverage_path_planning_heuristic.h"
#include "sspp/distance_heuristic.h"
#include "sspp/rviz_drawing_tools.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
//#include <component_test/mesh_surface.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/min_cut_segmentation.h>
using namespace SSPP;

int main( int argc, char **  argv)
{
    ros::init(argc, argv, "multi_agent_path_planning");
    ros::NodeHandle nh;

    //****************
    // ros publishers
    //****************
    ros::Publisher originalCloudPub  = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher visiblePub        = nh.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    ros::Publisher robotPosePub      = nh.advertise<geometry_msgs::PoseArray>("robot_pose", 10);
    ros::Publisher sensorPosePub     = nh.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);
    ros::Publisher robotPoseSSPub    = nh.advertise<geometry_msgs::PoseArray>("SS_robot_pose", 10);
    std::vector<ros::Publisher> sensorsPoseSSPub;
    std::vector<ros::Publisher> pathPubVec;
    std::vector<ros::Publisher> connectionsPubVec;
    ros::Publisher octomapPub        = nh.advertise<octomap_msgs::Octomap>("octomap", 1);


    //*********************************************************************
    // load pcd files and set parameters of robot, sensors and path planner
    //*********************************************************************
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (ros::package::getPath("component_test")+"/src/pcd/etihad_nowheels_nointernal_scaled_newdensed_lefted.pcd", *originalCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr coveredCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    OcclusionCullingGPU occlusionCulling(nh,"etihad_nowheels_nointernal_scaled_newdensed_lefted.pcd");

    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("map","/sspp_visualisation"));
    visualTools->deleteAllMarkers();
    visualTools->setLifetime(0.2);

    ros::Time timer_start = ros::Time::now();
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;

    PathPlanner * pathPlanner;

    double robotH=0.9,robotW=0.5,narrowestPath=0.987;
    double regGridConRad = 2.5;

    geometry_msgs::Point robotCenter;
    robotCenter.x = -0.3f;
    robotCenter.y = 0.0f;

    Robot *robot= new Robot("Robot",robotH,robotW,narrowestPath,robotCenter);
    Sensors sensor1(58,45,0.255,0.7,6.0,640,480,Vec3f(0,0.022,0.065), Vec3f(0,-0.349,0));
    Sensors sensor2(58,45,0.255,0.7,6.0,640,480,Vec3f(0,0.022,-0.065), Vec3f(0,0.349,0));
    std::vector<Sensors> sensors;
    sensors.push_back(sensor1);
    sensors.push_back(sensor2);

    // Every how many iterations to display the tree
    int progressDisplayFrequency = 1;
    pathPlanner = new PathPlanner(nh,robot,regGridConRad,progressDisplayFrequency,sensors);
    // This causes the planner to pause for the desired amount of time and display the search tree, useful for debugging
    pathPlanner->setDebugDelay(0.0);

    //*************************************************
    // segmentation according to the number of the agents
    // and the model length
    //*************************************************
    int numOfAgents = 3;
    double modelLength = 36;
    double lengthStart = -1*(modelLength/2);//assuming the center is at middle and the ground
    double lengthIncrement = modelLength/numOfAgents ;

    int maxCloudSizePerA = originalCloudPtr->points.size()/numOfAgents;
    std::cout<<"\n\nSize of cloud per agent: "<<maxCloudSizePerA<<std::endl;
    std::vector<geometry_msgs::PoseArray> sensorsPoseSS;
    geometry_msgs::PoseArray robotPoseSS;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
    int color=1;
    int r,g,b;

    //*****************************
    // loop through the segments
    //*****************************
    std::vector<visualization_msgs::Marker> pathSegVec,connectionsVec;
    for(int i=0; i<numOfAgents; i++)
    {

        pathPlanner->globalCloud.erase(pathPlanner->globalCloud.begin(),pathPlanner->globalCloud.end());
        pathPlanner->accuracyClusters.erase(pathPlanner->accuracyClusters.begin(),pathPlanner->accuracyClusters.end());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::IndicesPtr indices (new std::vector <int>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (originalCloudPtr);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (lengthStart, lengthStart+lengthIncrement);
        pass.filter (*cloud_filtered);

        pcl::PCDWriter writer;
        std::string cloudName = "cloud_cluster_"+boost::lexical_cast<std::string>(i)+".pcd";
        std::string cloudPath = ros::package::getPath("component_test")+"/src/pcd/";
        std::string cloudStr = cloudPath + cloudName;
        std::cout<<"cloud Name: "<<cloudName<<std::endl;
        writer.write<pcl::PointXYZ> (cloudStr, *cloud_filtered, false);

        //****************************************************
        //choose the coverage heuristic you want
        //and set the params and assign it to the path planner
        //****************************************************
        double coverageTolerance=1.0, targetCov=98.8;
        std::string collisionCheckModelPath = ros::package::getPath("component_test") + "/src/mesh/etihad_nowheels_nointernal_scaled_new_lefted.obj";
        std::string occlusionCullingModelName = cloudName;
        CoveragePathPlanningHeuristic coveragePathPlanningHeuristic(nh,collisionCheckModelPath,occlusionCullingModelName,false, true, InfoGainVolumetricH);
        coveragePathPlanningHeuristic.setCoverageTarget(targetCov);
        coveragePathPlanningHeuristic.setCoverageTolerance(coverageTolerance);
        pathPlanner->setHeuristicFucntion(&coveragePathPlanningHeuristic);

        //**************************************
        //      search space generation
        //**************************************
        gridStartPose.position.x = -18 ;//-18
        gridStartPose.position.y = lengthStart ;//-25
        gridStartPose.position.z = 9 ;//1
        gridSize.x = 36;//36
        gridSize.y = int(lengthIncrement);//50
        gridSize.z = 15;//15

        std::cout<<"grid generation in progress\n"<<"grid Size: " <<gridSize.x<<" "<<gridSize.y<<" "<<gridSize.z<<std::endl;
        std::cout<<"grid start: " <<gridStartPose.position.x<<" "<<gridStartPose.position.y<<" "<<gridStartPose.position.z<<std::endl;

        pathPlanner->dynamicNodesGenerationAndConnection(gridStartPose,gridSize,4.5,1.5);
        std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
        visualization_msgs::Marker connectionsMarker = drawLines(searchSpaceConnections,10000,color,100000000,0.03);
        ros::Publisher  connPub1   = nh.advertise<visualization_msgs::Marker>("connections_"+boost::lexical_cast<std::string>(i), 10);
        connectionsPubVec.push_back(connPub1);
        connectionsVec.push_back(connectionsMarker);
        std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
        std::cout<<"\n\n---->>> Total Nodes in search Space ="<<searchSpaceNodes.size()<<std::endl;

        pathPlanner->getRobotSensorPoses(robotPoseSS,sensorsPoseSS);
        coveragePathPlanningHeuristic.setMaxMinSensorAccuracy(sensorsPoseSS);

        //TODO: search space filtering in the connection part between the segments properly

        //*******************************************
        //       coloring the segmented part
        //*******************************************
        if(color == 1)
        {
            r = 255.0;
            g = 0.0;
            b = 0.0;
        }
        else if(color == 2)
        {
            r = 0.0;
            g = 255.0;
            b = 0.0;
        }
        else if(color == 3)
        {
            r = 0.0;
            g = 0.0;
            b = 255.0;
        }
        else
        {
            if(r==255)
                r=0;
            else r += 50;
            if(g==255)
                g=0;
            else g += 50;
            if(b==255)
                b=0;
            else b += 50;
        }

        for(int j=0; j<cloud_filtered->points.size(); j++)
        {
            pcl::PointXYZRGB point = pcl::PointXYZRGB(r,g,b);
            point.x = cloud_filtered->points[j].x;
            point.y = cloud_filtered->points[j].y;
            point.z = cloud_filtered->points[j].z;
            segmentedCloudPtr->points.push_back(point);
        }
        color++;

        //***************************************************
        //update the search space discretization/path planner
        // starting point  for the next segment
        //***************************************************
        lengthStart = lengthStart+lengthIncrement;

        //*****************************
        //      path planning
        //*****************************
        ros::Time timer_restart = ros::Time::now();
        Pose start(30.0,lengthStart,20,DTOR(0.0));
        Node * path = pathPlanner->startSearch(start);
        ros::Time timer_end = ros::Time::now();
        std::cout<<"\nPath Finding took:"<<double(timer_end.toSec() - timer_restart.toSec())<<" secs";

        //path print and visualization
        if(path)
        {
            //uncomment if you want to print all the nodes of the generated path
            //pathPlanner->printNodeList();
        }
        else
        {
            std::cout<<"\nNo Path Found";
        }
        std::cout<<"\nPath Finding took:"<<double(timer_end.toSec() - timer_restart.toSec())<<" secs";

        //*************************************************
        //write to file and visualize the generated path
        //and find the properties of the path
        //*************************************************
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
        cc <<i;
        std::string file_loc = ros::package::getPath("sspp")+"/resources/"+ss.str()+"%path_multi_"+occlusionCullingModelName+"GPU_Dynamic_dsscp_seg_"+cc.str()+".txt";
        pathFile.open (file_loc.c_str());
        octomap::OcTree* oct;
        std::vector<double> accuracyPerViewpointAvg;
        double accuracySum = 0;
        while(path !=NULL)
        {
            tf::Quaternion qt(path->pose.p.orientation.x,path->pose.p.orientation.y,path->pose.p.orientation.z,path->pose.p.orientation.w);
            yaw = tf::getYaw(qt);
            pathFile << path->pose.p.position.x<<" "<<path->pose.p.position.y<<" "<<path->pose.p.position.z<<" "<<yaw<<"\n";
            pcl::PointCloud<pcl::PointXYZ> temp;

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
                    temp += temp_cloud;

                    if(temp.points.size()!=0)
                    {
                        double avgAcc = occlusionCulling.calcAvgAccuracy(temp_cloud,path->next->senPoses[i].p);
                        double a = (occlusionCulling.maxAccuracyError - avgAcc)/occlusionCulling.maxAccuracyError;
                        accuracyPerViewpointAvg.push_back(a);
                        accuracySum += avgAcc;
                        //std::cout<<"accuracy per viewpoints: "<<a<<" "<<avgAcc<<std::endl;
                    }
                }


                pathSegments.push_back(linePoint);

                dist=dist+ Dist(path->next->pose.p,path->pose.p);
            }else{
                if(coveragePathPlanningHeuristic.getHeuristicType() == InfoGainVolumetricH)
                    oct = new octomap::OcTree(*path->octree);
            }
            path = path->next;
        }
        pathFile.close();
        visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"generated_path");
        visualization_msgs::Marker pathMarker = drawLines(pathSegments,20000,color,10000000,0.1);
        coveredCloudPtr->points=combined.points;

        //pathPub.publish(pathMarker);
        ros::Publisher  pathPub1   = nh.advertise<visualization_msgs::Marker>("path_"+boost::lexical_cast<std::string>(i), 10);
        pathPubVec.push_back(pathPub1);
        pathSegVec.push_back(pathMarker);
        std::cout<<"\nDistance calculated from the path: "<<dist<<"m\n";
        std::cout<<"Covered Cloud % : "<<occlusionCulling.calcCoveragePercent(coveredCloudPtr)<<"%\n";
        std::cout<<"Average Accuracy per viewpoint is "<<accuracySum/accuracyPerViewpointAvg.size()<<std::endl;

        //*************************************************
        // path planning in the next segment with the new
        // search space, avoid acumelating previous search
        // space with the new one of the next segment
        //*************************************************
        pathPlanner->freeSearchSpace();

    }


    ros::Rate loopRate(10);
    for(int i = 0; i<sensorsPoseSS.size(); i++)
    {
        ros::Publisher  sensorPoseSSPub   = nh.advertise<geometry_msgs::PoseArray>("SS_sensor_pose_"+boost::lexical_cast<std::string>(i), 10);
        sensorsPoseSSPub.push_back(sensorPoseSSPub);
    }
    sensor_msgs::PointCloud2 cloud1,cloud2;
    while (ros::ok())
    {

        //if( coveragePathPlanningHeuristic.getHeuristicType() == InfoGainVolumetricH)
        //{
        //    octomap_msgs::Octomap octomap ;
        //    octomap.binary = 1 ;
        //    octomap.id = 1 ;
        //    octomap.resolution =0.25;
        //    octomap.header.frame_id = "map";
        //    octomap.header.stamp = ros::Time::now();
        //    bool res = octomap_msgs::fullMapToMsg(*oct, octomap);
        //    if(res)
        //    {
        //        octomapPub.publish(octomap);
        //    }
        //    else
        //    {
        //        ROS_WARN("OCT Map serialization failed!");
        //    }
        //}

        pcl::toROSMsg(*originalCloudPtr, cloud1); //cloud of original (white) using original cloud
        cloud1.header.stamp = ros::Time::now();
        cloud1.header.frame_id = "map"; //change according to the global frame please!!
        originalCloudPub.publish(cloud1);

        pcl::toROSMsg(*segmentedCloudPtr, cloud2); //cloud of segments
        cloud2.header.stamp = ros::Time::now();
        cloud2.header.frame_id = "map"; //change according to the global frame please!!
        visiblePub.publish(cloud2);

        //robotPose.header.frame_id= "map";
        //robotPose.header.stamp = ros::Time::now();
        //robotPosePub.publish(robotPose);

        //sensorPose.header.frame_id= "map";
        //sensorPose.header.stamp = ros::Time::now();
        //sensorPosePub.publish(sensorPose);

        robotPoseSS.header.frame_id= "map";
        robotPoseSS.header.stamp = ros::Time::now();
        robotPoseSSPub.publish(robotPoseSS);

        for(int i = 0 ; i<sensorsPoseSS.size(); i++)
        {
            sensorsPoseSS[i].header.frame_id= "map";
            sensorsPoseSS[i].header.stamp = ros::Time::now();
            sensorsPoseSSPub[i].publish(sensorsPoseSS[i]);
        }

        for(int i = 0 ; i<pathSegVec.size(); i++)
        {
            pathPubVec[i].publish(pathSegVec[i]);
        }

        for(int i = 0 ; i<connectionsVec.size(); i++)
        {
            connectionsPubVec[i].publish(connectionsVec[i]);
        }
        ros::spinOnce();
        loopRate.sleep();
    }
    delete robot;
    delete pathPlanner;
    return 0;
}
