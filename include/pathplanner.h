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
#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <astar.h>
#include <QTime>
#include <stdio.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Segment_3.h>
#include <CGAL/AABB_triangle_primitive.h>
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"

using namespace fcl;

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line1;
typedef K::Point_3 Point;
typedef K::Segment_3 Segment;
typedef K::Triangle_3 CGALTriangle;
typedef std::list<CGALTriangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree1;

namespace SSPP
{
    class PathPlanner : public Astar
    {
    public :
        ros::NodeHandle nh;
        double  reg_grid_conn_rad;
    public :
        unsigned int getPlanningSteps();
        void   setConRad(double);
        void   freeResources();
        void   printNodeList ();
        void   generateRegularGrid(geometry_msgs::Pose gridStartPose, geometry_msgs::Vector3 gridSize);
        void   generateRegularGrid(geometry_msgs::Pose gridStartPose, geometry_msgs::Vector3 gridSize, float gridRes);
        void   generateRegularGrid(geometry_msgs::Pose gridStartPose,geometry_msgs::Vector3 gridSize, float gridRes, bool sampleOrientations);
        void   connectNodes();
        void   showConnections();
        void   showSearchSpace();
        visualization_msgs::Marker drawpoints(std::vector<geometry_msgs::Point> points);
        void   findRoot();
        void   freePath();
        void   loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::list<CGALTriangle>& triangles);
        PathPlanner(ros::NodeHandle & nh, Robot *, double reg_grid_conn_rad);
        ~PathPlanner();
        void loadRegularGrid(const char *filename1, const char *filename2);
        bool sampleOrientations;
        /*!
         * \brief gridResolution
         * Grid Resolution in meters used while generating the samples
         */
        float gridResolution;
        /*!
         * \brief orientationResolution
         * Orientation resolution in degrees used while generating the samples
         */
        float orientationResolution;
    private:
	ros::Publisher searchSpacePub;
	std::vector<geometry_msgs::Point> pts;
    std::list<CGALTriangle> triangles;
    std::vector<Vec3f> p1;
    std::string pathDir;
    Tree1* tree_cgal;
};

}

#endif /*PATHPLANNER_H_*/
