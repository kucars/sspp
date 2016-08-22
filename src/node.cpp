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
#include "node.h"

namespace SSPP
{

Node :: Node ():
    nearest_obstacle(0.0),
    g_value(0.0),
    h_value(0.0),
    f_value(0.0),
    gain_value(0.0),
    distance(0.0),
    coverage(0.0)
{
    parent = next = prev = NULL;
    cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
}

Node :: ~Node ()
{
    parent = next = prev = NULL;
}

bool Node ::operator == (Node a)
{
    return ( isPositionEqual(this->pose.p.position,a.pose.p.position)  && isOrientationEqual(this->pose.p.orientation,a.pose.p.orientation));
}

bool Node ::operator != (Node a)
{
    return ( !(isPositionEqual(this->pose.p.position,a.pose.p.position) && isOrientationEqual(this->pose.p.orientation,a.pose.p.orientation)));
}

}
