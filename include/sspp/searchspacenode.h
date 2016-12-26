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
#ifndef SEARCHSPACENODE_H_
#define SEARCHSPACENODE_H_

#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseArray.h>

using namespace std;

enum {BridgeNode,RegGridNode};

namespace SSPP
{

class SearchSpaceNode
	{
	public :
        geometry_msgs::Pose location;//uavposition TODO: change it later to uavLocation
        geometry_msgs::PoseArray sensorLocation;//sensorposition
        SearchSpaceNode * next;
		int type,id;
        std::vector <SearchSpaceNode *>  children;
		 SearchSpaceNode ();
		~SearchSpaceNode ();
	};
}

#endif /*SEARCHSPACENODE_H_*/
