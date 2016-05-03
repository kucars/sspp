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
#ifndef DIST_HEURISTICT_H_
#define DIST_HEURISTICT_H_

#include <QString>
#include <QHash>
#include "ssppexception.h"
#include "node.h"
#include "heuristic_interface.h"
#include "rviz_drawing_tools.h"

using namespace std;

namespace SSPP
{

class DistanceHeuristic:public Heuristic
{
public:
    DistanceHeuristic(ros::NodeHandle &n, bool d=false);
    ~DistanceHeuristic(){}
    double gCost(Node *node);
    double hCost(Node *node);
    bool isCost();
    void setDebug(bool debug);
    void setEndPose(geometry_msgs::Pose pose);
    void setTolerance2Goal(double tolerance2Distance);
    void calculateHeuristic(Node *n);
    bool terminateConditionReached(Node *node);
    bool isConnectionConditionSatisfied(SearchSpaceNode*temp, SearchSpaceNode*S);
    bool isFilteringConditionSatisfied(geometry_msgs::Pose pose, geometry_msgs::PoseArray &correspondingSensorPoses, double minDist, double maxDist);
    void displayProgress(vector<Tree> tree);
private:
    bool debug;
    geometry_msgs::Pose endPose;
    double tolerance2Goal;
    ros::Publisher treePub;
};

}
#endif /*DIST_HEURISTICT_H_*/
