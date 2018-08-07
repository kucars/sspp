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
#ifndef SENSORS_H_
#define SENSORS_H_
#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <deque>
#include <std_msgs/Bool.h>
#include <cmath>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cassert>
#include <iomanip>
//#include "fcl/math/transform.h"
//using namespace fcl;
using namespace std;
class Sensors
{
public :
    std::string robotModel,robotName;
    double hFOV, vFOV, focalLength, near, far, resWidth, resHeight;// sensorSize;
//    geometry_msgs::Pose sensorLocation; //with respect to the UAV
    Eigen::Vector3f sensorPose, sensorRPY;
    Sensors(double hfov, double vfov, double fl, double nearR, double farR, double width, double height, Eigen::Vector3f sP, Eigen::Vector3f sRPY);
    geometry_msgs::Pose robot2sensorTransformation(geometry_msgs::Pose pose);
    Sensors();
    ~Sensors();
};

#endif /*SENSORS_H_*/
