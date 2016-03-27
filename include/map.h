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
#ifndef MAP_H_
#define MAP_H_
#include "utils.h"
#include <QByteArray>
#include <ostream>
#include <cstdio>

class QObject;

class Map
{
public:
    Map();
    ~Map();
    Map(int width, int height,double mapRes,QPointF center,Pose p);
    Map(int width, int height, double resolution,  QByteArray rawData);
    Map(float mapRes,Pose p);
    Map(Pose p);
    Map * clone();
    void savePgm();
    void scale(int newWidth,int newHeight);
    // transfers from pixel coordinate to the main coordinate system
    void convertPix(geometry_msgs::Pose *p);
    // transfers from main coordinate to the pixel coordinate system
    void convert2Pix(geometry_msgs::Pose *p);
    int width, height;
    float mapRes;
    Pose global_pose;
    // for OG-Maps
    QByteArray rawData;
    // for Planners
    bool    ** grid, **temp;
    QVector <QPointF> pointCloud;
    // Axis Center of the Map
    QPointF center;
};
#endif /*MAP_H_*/
