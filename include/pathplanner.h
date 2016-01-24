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

static const unsigned int BRIDGE_TEST       = 1;
static const unsigned int REGULAR_GRID      = 2;
static const unsigned int NODES_CONNECT     = 4;
static const unsigned int OBST_PENALTY      = 8;
static const unsigned int OBST_EXPAND       = 16;

namespace SSPP
{
    class PathPlanner : public Astar
    {
    public :
        ros::NodeHandle nh;
        bool map_initialized;
        double  obstacle_expansion_radius,bridge_length,bridge_res,
                reg_grid_conn_rad,obst_penalry_radius,bridge_conn_rad;
    public :
        unsigned int getPlanningSteps();

        void   setConRad(double);
        void   setObstDist(double);
        void   freeResources();
        void   printNodeList ();

        void   generateRegularGrid(const char *filename1, const char *filename2);
        void   connectNodes();
        void   showConnections();
//        void   showSearchSpace();
//        void   displayChildren(SearchSpaceNode *temp);

//	visualization_msgs::Marker drawpoints(std::vector<geometry_msgs::Point> points);

        void   determineCheckPoints();
        void   findRoot();
        void   freePath();
//        bool   checkShortestDistance(geometry_msgs::Pose p, double neigbhour_pixel_distance);
        PathPlanner(ros::NodeHandle & nh, Robot *,double dG,double cT,double reg_grid_conn_rad);
        ~PathPlanner();
    private:
        unsigned int planningSteps;
        bool obstaclesExpanded;
	ros::Publisher searchSpacePub;
	std::vector<geometry_msgs::Point> pts;

};

}

#endif /*PATHPLANNER_H_*/
