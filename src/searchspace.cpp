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
    geometry_msgs::PoseArray correspondingSensorsPoses;
    correspondingSensorPose.position.x = 0; correspondingSensorPose.position.y = 0; correspondingSensorPose.position.z = 0;
    correspondingSensorPose.orientation.x = 0; correspondingSensorPose.orientation.y = 0; correspondingSensorPose.orientation.z = 0; correspondingSensorPose.orientation.w = 0;
    correspondingSensorsPoses.poses.push_back(correspondingSensorPose);
    return insertNode(nodePose,correspondingSensorsPoses,idCount++);
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose nodePose,int id)
{
    geometry_msgs::Pose correspondingSensorPose;
    geometry_msgs::PoseArray correspondingSensorsPoses;
    correspondingSensorPose.position.x = 0; correspondingSensorPose.position.y = 0; correspondingSensorPose.position.z = 0;
    correspondingSensorPose.orientation.x = 0; correspondingSensorPose.orientation.y = 0; correspondingSensorPose.orientation.z = 0; correspondingSensorPose.orientation.w = 0;
    correspondingSensorsPoses.poses.push_back(correspondingSensorPose);
    return insertNode(nodePose,correspondingSensorsPoses,id);
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose nodePose, geometry_msgs::PoseArray correspondingSensorsPoses)
{
    return insertNode(nodePose,correspondingSensorsPoses,idCount++);
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose nodePose, geometry_msgs::PoseArray correspondingSensorsPoses, int id)
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
            for(int i=0; i<correspondingSensorsPoses.poses.size(); i++)
            {
                temp->sensorLocation.poses.push_back(correspondingSensorsPoses.poses[i]);
            }
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
            for(int i=0; i<correspondingSensorsPoses.poses.size(); i++)
            {
                temp->sensorLocation.poses.push_back(correspondingSensorsPoses.poses[i]);
            }
            temp->next   = searchspace;
            temp->type   = RegGridNode;
            temp->id 	 = id;
            searchspace  = temp;
        }
    }
    return temp;
}

SearchSpaceNode * SearchSpace::insertTempSearchSpace( geometry_msgs::PoseArray robotPoses, std::vector<geometry_msgs::PoseArray> correspondingSensorsPoses)
{
    SearchSpaceNode *temp;
    SearchSpaceNode * space =NULL;
    for(int i =0; i<robotPoses.poses.size(); i++)
    {
        if(!nodeExists(space,robotPoses.poses[i]))
        {
            // Constructing the ROOT NODE
            if (space == NULL )
            {
                temp = new SearchSpaceNode;
                temp->location.position          = robotPoses.poses[i].position;
                temp->location.orientation       = robotPoses.poses[i].orientation;
                for(int j=0; j<correspondingSensorsPoses.size(); j++)
                {
                    temp->sensorLocation.poses.push_back(correspondingSensorsPoses[j].poses[i]);
                }
                temp->next     = NULL;
                temp->type     = RegGridNode;
                //  temp->id 	   = id; //already inserted in the main searchspace
                space    = temp;
            }
            else
            {
                temp = new SearchSpaceNode;
                temp->location.position          = robotPoses.poses[i].position;
                temp->location.orientation       = robotPoses.poses[i].orientation;
                for(int j=0; j<correspondingSensorsPoses.size(); j++)
                {
                    temp->sensorLocation.poses.push_back(correspondingSensorsPoses[j].poses[i]);
                }
                temp->next   = space;
                temp->type   = RegGridNode;
                //  temp->id 	 = id; //already inserted in the main searchspace
                space  = temp;
            }
        }
    }
    return temp;
}
SearchSpaceNode * SearchSpace::nodeExists(SearchSpaceNode * space, geometry_msgs::Pose nodePose)
{
    SearchSpaceNode *temp = space;

    while (temp != NULL)
    {

        if(samePosition(nodePose,temp->location))
            return temp;

        temp = temp->next;
    }
    // node with the given location does not exist
    return NULL;
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

    // Empty 
    if(!temp)
        return false;

    // if it's the first node, simply remove it
    if(samePosition(nodePose,temp->location))
    {
        searchspace = temp->next;
        delete temp;
        return true;
    }

    while (temp->next != NULL)
    {
        if(samePosition(nodePose,temp->next->location))
        {
            SearchSpaceNode *toDelete = temp->next;
            temp->next = temp->next->next;
            delete toDelete;
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
void SearchSpace::freeTempSearchSpace(SearchSpaceNode * space)
{
    SearchSpaceNode *temp;
    while (space != NULL)
    {
        temp = space;
        space = space->next;
        delete temp;
    };
}


}
