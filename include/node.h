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
#ifndef NODE_H_
#define NODE_H_

#include<QPointF>
#include "robot.h"
#include "utils.h"
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Triangle_3.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel exactKernel;
typedef CGAL::Triangle_3<exactKernel> Triangle_3;
typedef std::vector<Triangle_3> Triangles;

namespace SSPP
{
/* This Class Represents the Node Structure in the search Tree,
 * each node encapsulates information about it's parent, next
 * and previous node in the list, it's location, travelling and
 * herustic costs.
 */
class Node
{
public :
    int id,depth,direction;
    double nearest_obstacle,g_value,h_value,f_value,distance,coverage;
    Triangles surfaceTriangles;
    Node  * parent, * next, * prev;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> voxels;

    Pose   pose;
    std::vector<Pose>   senPoses;
    Node();
    bool operator == (Node);
    bool operator != (Node);
    ~Node();
};

}

#endif /*NODE_H_*/
