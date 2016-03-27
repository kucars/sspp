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
#ifndef ASTAR_H_
#define ASTAR_H_

#include <QPointF>
#include <QHash>
#include <QBitArray>
#include <QVector>
#include <QImage>
#include <QString>
#include <math.h>
#include "llist.h"
#include "node.h"
#include "utils.h"
#include "robot.h"
#include "searchspace.h"
#include "heuristic_interface.h"
#include "ssppexception.h"
#include "map.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

enum{METRIC,PIXEL};
namespace SSPP
{

class Astar: public SearchSpace
{
private:
    void   findRoot() throw(SSPPException);
    void   findDest() throw(SSPPException);
    Node  *makeChildrenNodes(Node *parent) ;
    ros::NodeHandle  nh;
public:
    Astar(ros::NodeHandle & n, Robot *);
    Astar();
    virtual ~Astar();
    void setRobot(Robot *);
    void setHeuristicFucntion(Heuristic *heuristicFun);
    long int MAXNODES;
    Heuristic *heuristic;
    bool debug;
    Pose start;
    int globalcount;
    Robot *robot;
    Node *root, *dest, *current, *childList, *curChild, *q, * test,*path, *p;
    LList *openList,*closedList;
    vector <Tree> tree;
    void displayTree();

    visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color, int duration, double scale);
    visualization_msgs::Marker drawPoints(std::vector<geometry_msgs::Point> points, int c_color, int duration);
    void freeNode   (Node *);
    bool goalReached(Node *n);
    Node*  startSearch  (Pose start);
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint;
    ros::Publisher treePub;
    ros::Publisher connectionPub;
    ros::Publisher pathPub;
    ros::Publisher pathPointPub;
    ros::Publisher testPointPub;
    ros::Publisher coveredPointsPub;
    ros::Publisher connectionDebugPub;

};

}

#endif /*ASTAR_H_*/
