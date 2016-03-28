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
#include "searchspace.h"

namespace SSPP
{

SearchSpace::SearchSpace():
    searchspace(NULL),
    idCount(0)
{
}

SearchSpace::~SearchSpace()
{
    SearchSpaceNode *temp;
    while (searchspace != NULL)
    {
        temp = searchspace;
        searchspace = searchspace->next;
        delete temp;
    }
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose nodePose)
{
    geometry_msgs::Pose correspondingSensorPose;
    correspondingSensorPose.position.x = 0; correspondingSensorPose.position.y = 0; correspondingSensorPose.position.z = 0;
    correspondingSensorPose.orientation.x = 0; correspondingSensorPose.orientation.y = 0; correspondingSensorPose.orientation.z = 0; correspondingSensorPose.orientation.w = 0;
    return insertNode(nodePose,correspondingSensorPose,idCount++);
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose nodePose,int id)
{
    geometry_msgs::Pose correspondingSensorPose;
    correspondingSensorPose.position.x = 0; correspondingSensorPose.position.y = 0; correspondingSensorPose.position.z = 0;
    correspondingSensorPose.orientation.x = 0; correspondingSensorPose.orientation.y = 0; correspondingSensorPose.orientation.z = 0; correspondingSensorPose.orientation.w = 0;
    return insertNode(nodePose,correspondingSensorPose,id);
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose nodePose, geometry_msgs::Pose correspondingSensorPose)
{
    return insertNode(nodePose,correspondingSensorPose,idCount++);
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose nodePose, geometry_msgs::Pose correspondingSensorPose, int id)
{
    SearchSpaceNode *temp;
    if(!nodeExists(nodePose))
    {
        // Constructing the ROOT NODE
        if (searchspace == NULL )
        {
            temp = new SearchSpaceNode;
            temp->location.position          = nodePose.position;
            temp->location.orientation       = nodePose.orientation;
            temp->sensorLocation.position    = correspondingSensorPose.position;
            temp->sensorLocation.orientation = correspondingSensorPose.orientation;
            temp->parent   = NULL;
            temp->next     = NULL;
            temp->type     = RegGridNode;
            temp->id 	   = id;
            searchspace    = temp;
        }
        else
        {
            temp = new SearchSpaceNode;
            temp->location.position          = nodePose.position;
            temp->location.orientation       = nodePose.orientation;
            temp->sensorLocation.position    = correspondingSensorPose.position;
            temp->sensorLocation.orientation = correspondingSensorPose.orientation;
            temp->parent = NULL;
            temp->next   = searchspace;
            temp->type   = RegGridNode;
            temp->id 	 = id;
            searchspace  = temp;
        }
    }
    return temp;
}

SearchSpaceNode * SearchSpace::nodeExists(geometry_msgs::Pose nodePose)
{
    SearchSpaceNode *temp = searchspace;
    while (temp != NULL)
    {
        if(samePosition(nodePose,temp->location))
            return temp;
        temp = temp->next;
    }
    // node with the given location does not exist
    return NULL;
}

bool SearchSpace::removeNode(geometry_msgs::Pose nodePose)
{
    SearchSpaceNode *temp = searchspace;
    while (temp != NULL)
    {
        if(samePosition(nodePose,temp->location))
        {
            temp->parent->next = temp->next;
            delete temp;
            return true;
        }
        temp = temp->next;
    }
    // Not found in the list
    return false;
}

void SearchSpace:: freeSearchSpace()
{
    SearchSpaceNode *temp;
    while (searchspace != NULL)
    {
        temp = searchspace;
        searchspace = searchspace->next;
        delete temp;
    };
}

}
