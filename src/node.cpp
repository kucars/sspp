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
#include "node.h"

namespace SSPP
{

Node :: Node ():
    nearest_obstacle(0.0),
    g_value(0.0),
    h_value(0.0),
    f_value(0.0)
{
    parent = next = prev = NULL;
//    obj = new OcclusionCulling("scaled_desktop.pcd");
    cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
}

Node :: ~Node ()
{
    parent = next = prev = NULL;
}

bool Node ::operator == (Node a)
{
    return ( isEqual(this->pose.p.position.x,a.pose.p.position.x) && isEqual(this->pose.p.position.y,a.pose.p.position.y) && isEqual(this->pose.p.position.z,a.pose.p.position.z));
}

bool Node ::operator != (Node a)
{
    return ( !isEqual(this->pose.p.position.x,a.pose.p.position.x) || !isEqual(this->pose.p.position.y,a.pose.p.position.y) || !isEqual(this->pose.p.position.z,a.pose.p.position.z));
}

}
