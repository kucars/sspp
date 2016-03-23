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
#include "searchspace.h"

namespace SSPP
{

SearchSpace::SearchSpace():
search_space(NULL)
{
}

SearchSpace::~SearchSpace()
{
    SearchSpaceNode *temp;
    while (search_space != NULL)
    {
        temp = search_space;
        search_space = search_space->next;
        delete temp;
    }
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose loc,int id)
{
    SearchSpaceNode *temp;
    if(!(temp=nodeExists(loc)))
    {
        if (search_space == NULL ) // Constructing the ROOT NODE
        {
            temp = new SearchSpaceNode;
            temp->location.position.x = loc.position.x;
            temp->location.position.y = loc.position.y;
            temp->location.position.z = loc.position.z;
            temp->location.orientation.x = loc.orientation.x;//newly added
            temp->location.orientation.y = loc.orientation.y;//newly added
            temp->location.orientation.z = loc.orientation.z;//newly added
            temp->location.orientation.w = loc.orientation.w;//newly added
//            temp->obstacle_cost = 0;
            temp->parent   	= NULL;
            temp->next     	= NULL;
            temp->id 	= id;
            search_space 	= temp;
        }
        else
        {
            temp = new SearchSpaceNode;
            temp->location.position.x = loc.position.x;
            temp->location.position.y = loc.position.y;
            temp->location.position.z = loc.position.z;
            temp->location.orientation.x = loc.orientation.x;//newly added
            temp->location.orientation.y = loc.orientation.y;//newly added
            temp->location.orientation.z = loc.orientation.z;//newly added
            temp->location.orientation.w = loc.orientation.w;//newly added
//            temp->obstacle_cost = 0;
            temp->parent 	= NULL;
            temp->next   	= search_space;
            temp->id 	= id;
            search_space 	= temp;
        }
    }
    return temp;
}

SearchSpaceNode * SearchSpace::insertNode(geometry_msgs::Pose loc)
{
    SearchSpaceNode *temp;
    if(!(temp=nodeExists(loc)))
    {
        if (search_space == NULL ) // Constructing the ROOT NODE
        {
            temp = new SearchSpaceNode;
            temp->location.position.x = loc.position.x;
            temp->location.position.y = loc.position.y;
            temp->location.position.z = loc.position.z;
            temp->location.orientation.x = loc.orientation.x;//newly added
            temp->location.orientation.y = loc.orientation.y;//newly added
            temp->location.orientation.z = loc.orientation.z;//newly added
            temp->location.orientation.w = loc.orientation.w;//newly added
//            temp->obstacle_cost = 0;
            temp->parent   = NULL;
            temp->next     = NULL;
            search_space = temp;
        }
        else
        {
            temp = new SearchSpaceNode;
            temp->location.position.x = loc.position.x;
            temp->location.position.y = loc.position.y;
            temp->location.position.z = loc.position.z;
            temp->location.orientation.x = loc.orientation.x;//newly added
            temp->location.orientation.y = loc.orientation.y;//newly added
            temp->location.orientation.z = loc.orientation.z;//newly added
            temp->location.orientation.w = loc.orientation.w;//newly added
//            temp->obstacle_cost = 0;
            temp->parent = NULL;
            temp->next   = search_space;
            search_space = temp;
        }
    }
    return temp;
}

SearchSpaceNode * SearchSpace::nodeExists(geometry_msgs::Pose loc)
{
    SearchSpaceNode *temp = search_space;
    while (temp != NULL)
    {
        if(samePosition(loc,temp->location))
            return temp;
        temp = temp->next;
    }
    // node with the given location does not exist
    return NULL;
}

bool SearchSpace::removeNode(geometry_msgs::Pose loc)
{
    SearchSpaceNode *temp = search_space;
    while (temp != NULL)
    {
        if(samePosition(loc,temp->location))
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
    while (search_space != NULL)
    {
        temp = search_space;
        search_space = search_space->next;
        delete temp;
    };
}

}
