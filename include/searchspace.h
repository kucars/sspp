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
#ifndef SEARCHSPACE_H_
#define SEARCHSPACE_H_

#include "searchspacenode.h"
#include "utils.h"

namespace SSPP
{

class SearchSpace
{
public:
    SearchSpaceNode * searchspace;
    void freeSearchSpace();
    void freeTempSearchSpace(SearchSpaceNode * space);
    SearchSpaceNode * insertNode(geometry_msgs::Pose nodePose);
    SearchSpaceNode * insertNode(geometry_msgs::Pose nodePose, int id);
    SearchSpaceNode * insertNode(geometry_msgs::Pose nodePose, geometry_msgs::PoseArray correspondingSensorsPoses);
    SearchSpaceNode * insertNode(geometry_msgs::Pose nodePose, geometry_msgs::PoseArray correspondingSensorsPoses, int id);
    SearchSpaceNode * insertTempSearchSpace(geometry_msgs::PoseArray robotPoses, std::vector<geometry_msgs::PoseArray> correspondingSensorsPoses);
    SearchSpaceNode * nodeExists(geometry_msgs::Pose nodePose);
    SearchSpaceNode * nodeExists(SearchSpaceNode * space, geometry_msgs::Pose nodePose);
    bool              removeNode(geometry_msgs::Pose nodePose);
    SearchSpace();
    virtual ~SearchSpace();
    int idCount;
};

}

#endif /*SEARCHSPACE_H_*/
