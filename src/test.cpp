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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace SSPP;
//Map * provideMapOG(QString name,double res,bool negate,Pose mapPose);
visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links);
visualization_msgs::Marker drawpoints(std::vector<geometry_msgs::Point> points);
/*
class PathPlanningThread : public QThread
{
    Q_OBJECT
public:
    PathPlanningThread(){}
    ~PathPlanningThread(){}
    virtual void run()
    {
        QString result;
        PathPlanner * pathPlanner;
        bool negate = false;
        Pose start(12.9,1.3,0,DTOR(-127.304)),end(-6.6,7.4,0,DTOR(140.194));
        double robotH=0.9,robotW=0.5,narrowestPath=0.987,mapRes= 0.05;
        double distanceToGoal = 0.4,bridgeLen=2.5,bridgeRes=0.1,regGridLen=0.2,regGridConRad=0.4,obstPenalty=3.0,bridgeConRad=0.7;
        QPointF robotCenter(-0.3f,0.0f);
        Robot *robot= new Robot(QString("Robot"),robotH,robotW,narrowestPath,robotCenter);
        Map   *map  = provideMapOG("casarea_s.png",mapRes,negate,Pose(0,0,0,0));
        PathPlanner * pathPlanner = new PathPlanner(robot,distanceToGoal,bridgeLen,bridgeRes,regGridLen,regGridConRad,obstPenalty,bridgeConRad);;
        QTime timer;
        const char * filename = "SearchSpace.txt";
        pathPlanner->setMap(map);
        pathPlanner->expandObstacles();
        pathPlanner->generateRegularGrid();
        pathPlanner->bridgeTest();
        pathPlanner->addCostToNodes();
        pathPlanner->connectNodes();
        pathPlanner->saveSpace2File(filename);

        std::cout<<"\nSpace Generation took:"<<timer.elapsed()/double(1000.00)<<" secs";
        pathPlanner->showConnections();

        timer.restart();
        Node * retval = pathPlanner->startSearch(start,end,METRIC);
        std::cout<<"\nPath Finding took:"<<(timer.elapsed()/double(1000.00))<<" secs";
        if(retval)
        {
            pathPlanner->printNodeList();
        }
        else
        {
            std::cout<<"\nNo Path Found";
        }
        delete robot;
        delete map;
        delete pathPlanner;
        emit resultReady(result);
    }
signals:
    void resultReady(const QString &s)
    {

    }
};
*/
//Map * provideMapOG(QString name,double res,bool negate,Pose mapPose)
//{
//    QImage image;
//    if(!image.load(name, 0))
//    {
//        std::cout<<"\nError Loading Image";
//        exit(1);
//    }
//    Map * retval;
//    QPointF center(image.width()/2.0,image.height()/2.0);
//    retval = new Map(image.width(),image.height(),res,center,mapPose);
//    long int count=0;
//    for(int i=0;i<image.width();i++)
//    {
//        QRgb color;
//        for(int j=0;j<image.height();j++)
//        {
//            color = image.pixel(i,j);
//            double color_ratio = (qRed(color) + qGreen(color) + qBlue(color))/(3.0*255.0);
//            if(!negate)
//            {
//                // White color(255) is Free and Black(0) is Occupied
//                if (  color_ratio > 0.9)
//                    retval->grid[i][j]= false;
//                else
//                {
//                    retval->grid[i][j]= true;
//                    count++;
//                }
//            }
//            else
//            {
//                // White color(255) is Occupied and Black(0) is Free
//                if ( color_ratio < 0.1)
//                    retval->grid[i][j]= false;
//                else
//                    retval->grid[i][j]= true;
//            }
//        }
//    }
//    return retval;
//}

int main( int argc, char **  argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle n;
    ros::Publisher original_cloud = n.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher path_pub = n.advertise<visualization_msgs::Marker>("generated_path", 10);
    ros::Publisher searchSpace_pub = n.advertise<visualization_msgs::Marker>("search_space", 10);
    ros::Publisher connectivity_pub = n.advertise<visualization_msgs::Marker>("connections", 10);

    PathPlanner * pathPlanner;

    //display the aircraft point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path = ros::package::getPath("component_test");
    pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/src/pcd/scaled_desktop.pcd", *cloud);


//    bool negate = false;
    Pose start(0.0,1.0,-37,DTOR(-127.304)),end(-4.0,1.0,-19,DTOR(140.194));
    double robotH=0.9,robotW=0.5,narrowestPath=0.987;//is not changed
    double distanceToGoal = 2.4,regGridLen=1.0,regGridConRad=1.5;
    QPointF robotCenter(-0.3f,0.0f);
    Robot *robot= new Robot(QString("Robot"),robotH,robotW,narrowestPath,robotCenter);
    pathPlanner = new PathPlanner(n,robot,distanceToGoal,regGridLen,regGridConRad);
    QTime timer;
    const char * filename = "SearchSpace.txt";
    pathPlanner->generateRegularGrid(filename);//IMPORTANT
//    pathPlanner->showSearchSpace();//visualization (not working for some reason)
    pathPlanner->connectNodes();//IMPORTANT
    std::cout<<"\nSpace Generation took:"<<timer.elapsed()/double(1000.00)<<" secs";
    pathPlanner->showConnections();

    //******for visualizing the search space & connectivity********
    SearchSpaceNode *temp = pathPlanner->search_space;
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
    visualization_msgs::Marker linesList1 = drawLines(lineSegments1);

    //**************************************************************






//    timer.restart();
//    Node * retval = pathPlanner->startSearch(start,end,METRIC);
//    std::cout<<"\nPath Finding took:"<<(timer.elapsed()/double(1000.00))<<" secs";
//    //path print and visualization
//    if(retval)
//    {
//        pathPlanner->printNodeList();
//    }
//    else
//    {
//        std::cout<<"\nNo Path Found";
//    }

//    Node * p = pathPlanner->path;
//    std::vector<geometry_msgs::Point> lineSegments;
//    geometry_msgs::Point linePoint;
//    while(p !=NULL)
//    {
//        if (p->next !=NULL)
//        {
//            linePoint.x = p->pose.p.position.x;
//            linePoint.y = p->pose.p.position.y;
//            linePoint.z = p->pose.p.position.z;
//            lineSegments.push_back(linePoint);

//            linePoint.x = p->next->pose.p.position.x;
//            linePoint.y = p->next->pose.p.position.y;
//            linePoint.z = p->next->pose.p.position.z;
//            lineSegments.push_back(linePoint);
//        }
//        p = p->next;
//    }
//    visualization_msgs::Marker linesList = drawLines(lineSegments);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sensor_msgs::PointCloud2 cloud1;
        pcl::toROSMsg(*cloud, cloud1);
        cloud1.header.frame_id = "map";
        cloud1.header.stamp = ros::Time::now();
        original_cloud.publish(cloud1);

        ROS_INFO("Publishing Marker");
//        path_pub.publish(linesList);
        searchSpace_pub.publish(points_vector);
        connectivity_pub.publish(linesList1);
        ros::spinOnce();
        loop_rate.sleep();
    }
    delete robot;
//    delete map;
    delete pathPlanner;
    return 0;
}

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links)
{
    visualization_msgs::Marker linksMarkerMsg;
    linksMarkerMsg.header.frame_id="/map";
    linksMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    linksMarkerMsg.id = 0;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    linksMarkerMsg.scale.x = 0.01;
    linksMarkerMsg.action  = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime  = ros::Duration(10.0);
    std_msgs::ColorRGBA color;
    color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
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
