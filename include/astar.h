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
#include "rviz_drawing_tools.h"

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
    Astar(ros::NodeHandle & n, Robot *,int progressDisplayFrequency);
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
    Node *root, *dest, *current, *childList, *curChild, *q, *p;
    LList *openList,*closedList;
    vector <Tree> tree;
    void freeNode   (Node *);
    bool goalReached(Node *n);
    void setProgressDisplayFrequency(int progressDisplayFrequency);
    void setDebugDelay(double delay);
    int progressDisplayFrequency;
    Node*  astarSearch(Pose start);
    Node*  tempChildList;
    double debugDelay;

    ros::Publisher childPosePub;
    ros::Publisher childSensorsPub;
    ros::Publisher parentPosePub;
    ros::Publisher parenSensorsPub;
    ros::Publisher branchPub;
    ros::Publisher octomapChildPub;
    int nodeToBeVisNum, nodesCounter;

};

}

#endif /*ASTAR_H_*/
