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
#ifndef ROBOT_H_
#define ROBOT_H_

#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include "utils.h"
#include "ros/ros.h"

using namespace std;
class Robot
{
public :
    double robotLength,robotWidth,obstacleRadius,robotMass,robotMI,robotRadius,robotCenterX,robotCenterY,
    robotSpeed,robotTurnRate,expansionRadius,narrowestPathDist,safetyTolerance, startx,starty;
    //! Motion model
    std::string robotModel,robotName;
    //! Holds the Latest Robot Position
    Pose robotLocation;
    // Center of Rotation
    geometry_msgs::Point robotCenter;
    // For Rendering the Robot Rectangle
    std::vector<geometry_msgs::Point> local_edge_points, check_points;
    void setCheckPoints(double o_r);
    void setPose(Pose location);
    void setSpeed(double speed);
    void setTurnRate(double turnRate);
    void findR();
    int  readConfigs(ros::NodeHandle nh, string nameSpace);
    Robot(std::string name, double length, double width, double narrowDist, geometry_msgs::Point center, double safetyTolerance=0.05);
    Robot();
    ~Robot();
};

#endif /*ROBOT_H_*/
