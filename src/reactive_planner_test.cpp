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
#include <ros/ros.h>
#include "sspp/pathplanner.h"

#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "sspp/distance_heuristic.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include <octomap_world/octomap_manager.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sspp/sspp_srv.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reactive_planner_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  sspp::sspp_srv planningService;
  geometry_msgs::Pose start,end;

  ros::service::waitForService("sspp_planner",ros::Duration(10.0));
  ROS_INFO("Planning Service Provider Ready");
  ros::Rate rate(20.0);

  while (ros::ok())
  {
    start.position.x = -2.12;
    start.position.y =  1.2;
    start.position.z =  0.2;

    end.position.x = 2;
    end.position.y = 2;
    end.position.z = 0;

    planningService.request.header.stamp = ros::Time::now();
    planningService.request.header.seq = 1;
    planningService.request.header.frame_id = "world";
    planningService.request.start = start;
    planningService.request.end   = end;
    planningService.request.grid_start = start;

    if(ros::service::call("sspp_planner", planningService))
    {
      ROS_INFO("Path Found");
      for (int i = 0; i < planningService.response.path.size(); i++)
      {
        std::cout<<"Path x:"<<planningService.response.path[i].position.x
                <<" y:"<<planningService.response.path[i].position.y
                <<" z:"<<planningService.response.path[i].position.z
                <<"\n";

        tf::Pose pose;
        tf::poseMsgToTF(planningService.response.path[i], pose);
        double yaw = tf::getYaw(pose.getRotation());
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
      }
      break;
    }
    else
    {
      ROS_INFO("No Path Found or planner not ready!");
      ros::Duration(1.0).sleep();
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
