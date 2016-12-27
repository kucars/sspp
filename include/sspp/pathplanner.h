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
#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <sspp/astar.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Eigen>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Segment_3.h>
#include <CGAL/AABB_triangle_primitive.h>
//#include "fcl/octree.h"
//#include "fcl/traversal/traversal_node_octree.h"
//#include "fcl/broadphase/broadphase.h"
//#include "fcl/shape/geometric_shape_to_BVH_model.h"
//#include "fcl/math/transform.h"
//#include "fcl/shape/geometric_shapes.h"
//#include "fcl/narrowphase/narrowphase.h"
//#include "fcl/collision.h"
//#include "fcl/ccd/motion.h"
//#include "fcl/BV/AABB.h"
//#include "fcl/collision_object.h"

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <sspp/sensors.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

//using namespace fcl;
namespace SSPP
{
    class PathPlanner : public Astar
    {
    public :
        ros::NodeHandle nh;
        double  regGridConRadius;
    public :
        unsigned int getPlanningSteps();
        void   setConRad(double);
        void   setMultiAgentSupport(bool allowMultiAgentSupport);
        void   freeResources();
        void   printNodeList ();
        void   printPath(Node *path);
        void   printPath(int index);
        void   printLastPath();
        void   generateRegularGrid(geometry_msgs::Pose gridStartPose, geometry_msgs::Vector3 gridSize);
        void   generateRegularGrid(geometry_msgs::Pose gridStartPose, geometry_msgs::Vector3 gridSize, float gridRes);
        void   generateRegularGrid(geometry_msgs::Pose gridStartPose,geometry_msgs::Vector3 gridSize, float gridRes, bool sampleOrientations=false, float orientationRes=360, bool samplesFiltering=false, bool insertSearchSpace=true);
        void   connectNodes();
        void   connectClustersInternalNodes(SearchSpaceNode * space, double connRadius);
        void   dynamicNodesGenerationAndConnection(geometry_msgs::Pose gridStartPose, geometry_msgs::Vector3 gridSize, double startRes, double resDecrement); //for dynamic sampling
        void   connectToNN(pcl::PointCloud<pcl::PointXYZ> cloudHull1, pcl::PointCloud<pcl::PointXYZ> cloudHull2);
        void   disconnectNodes();
        void   blockPath(Node* path);
        Node * startSearch(Pose startPose);
        std::vector<geometry_msgs::Point> getConnections();
        std::vector<geometry_msgs::Point> getSearchSpace();
        void getRobotSensorPoses(geometry_msgs::PoseArray& robotPoses, geometry_msgs::PoseArray& sensorPoses);
        std::vector<Sensors> robotSensors;
        void   findRoot();
        void   freePath();
        PathPlanner(ros::NodeHandle & nh, Robot *, double regGridConRadius, int progressDisplayFrequency, std::vector<Sensors> &rSensors);
        PathPlanner(ros::NodeHandle & nh, Robot *, double regGridConRadius, int progressDisplayFrequency);
        ~PathPlanner();
        void loadRegularGrid(const char *filename1, const char *filename2, const char *filename3);
        bool sampleOrientations;
        bool samplesFiltering;
        bool multiAgentSupport;
        bool insertSearchSpace;
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
        pcl::PointCloud<pcl::PointXYZ> globalCloud;
        std::vector<pcl::PointCloud<pcl::PointXYZ> > accuracyClusters;
        geometry_msgs::PoseArray robotFilteredPoses;
        std::vector<geometry_msgs::PoseArray> sensorsFilteredPoses;
        std::vector<Node*> paths;
    private:
};

}

#endif /*PATHPLANNER_H_*/
