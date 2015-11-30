/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
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
#include <QPointF>
#include <QString>
#include <QVector>
#include <iostream>
#include <math.h>
#include "utils.h"
#include "configfile.h"
//#include "logger.h"

using namespace std;
class Robot
{
public :
    double robotLength,robotWidth,obstacleRadius,robotMass,robotMI,robotRadius,
    robotSpeed,robotTurnRate,expansionRadius,narrowestPathDist,safetyTolerance, startx,starty;
    int robotPort;
    //! Motion model
    QString robotModel,robotName,robotIp;
    //! Holds the Latest Robot Position
    Pose robotLocation;
    // Center of Rotation
    QPointF robotCenter;
    // For Rendering the Robot Rectangle
    QVector<QPointF> local_edge_points, check_points;
    void setCheckPoints(double o_r);
    void setPose(Pose location);
    void setSpeed(double speed);
    void setTurnRate(double turnRate);
    void findR();
    int  readConfigs(ConfigFile *cf,int secId);
    Robot(QString name,double length,double width,double narrowDist,QPointF center);
    Robot(ConfigFile *cf,int secId);
    Robot();
    ~Robot();
};

#endif /*ROBOT_H_*/
