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
#include "heuristic.h"
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
    Astar(ros::NodeHandle & n,Robot *,double dG, double cT, QString heuristicType);
    Astar();
    void setRobot(Robot *);
    long int MAXNODES;
    QString hType;
    double distGoal;
    double covTolerance;
    double orientation2Goal;
    Heuristic *heuristic;
    Map    * map;
    Pose start,end;
    double targetCov;
    Robot *robot;
    Node *root, *dest, *current, *childList, *curChild, *q, * test,*path, *p;
    LList *openList,*closedList;
    vector <Tree> tree;
    void displayTree();
    visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links);
    void setSocialReward(QHash<QString, int>*);
    void freeNode     (Node *);
//     int  inObstacle   (geometry_msgs::Pose p, double angle);
    bool goalReached  (Node *n);
    bool surfaceCoverageReached (Node *n);// newly added
//    Node*  startSearch  (Pose start,Pose end,int);
    Node*  startSearch  (Pose start,double targetCov,int);
    virtual ~Astar();
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point linePoint;
    ros::Publisher treePub;
};

}

#endif /*ASTAR_H_*/
