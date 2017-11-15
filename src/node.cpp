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
#include "sspp/node.h"

namespace SSPP
{

Node :: Node ():
    nearest_obstacle(0.0),
    g_value(0.0),
    h_value(0.0),
    f_value(0.0),
    totalEntroby(0.0),
    distance(0.0),
    coverage(0.0),
    coveredVoxelsNum(0),
    coveredVolume(0.0)
{
    parent = next = prev = NULL;
    cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
}

Node::Node(Node *n)
{
  parent = n->parent;
  next   = n->next;
  prev   = n->prev;
  depth  = n->depth;
  pose.p.position.x = n->pose.p.position.x;
  pose.p.position.y = n->pose.p.position.y;
  pose.p.position.z = n->pose.p.position.z;
  pose.p.orientation.x = n->pose.p.orientation.x;
  pose.p.orientation.y = n->pose.p.orientation.y;
  pose.p.orientation.z = n->pose.p.orientation.z;
  pose.p.orientation.w = n->pose.p.orientation.w;
  id = n->id;
  for(int i=0;i<n->senPoses.size();i++)
    senPoses.push_back(n->senPoses[i]);
  nearest_obstacle = n->nearest_obstacle;
  g_value = n->g_value;
  h_value = n->h_value;
  f_value = n->f_value;
  totalEntroby = n->totalEntroby;
  distance = n->distance;
  coverage = n->coverage;
  coveredVoxelsNum = n->coveredVoxelsNum;
  coveredVolume = n->coveredVolume;
  cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>);
}

Node :: ~Node ()
{
    parent = next = prev = NULL;
}

bool Node ::operator == (Node a)
{
    return ( isPositionEqual(this->pose.p.position,a.pose.p.position)  && isOrientationEqual(this->pose.p.orientation,a.pose.p.orientation));
}

bool Node ::operator != (Node a)
{
  return ( !(isPositionEqual(this->pose.p.position,a.pose.p.position) && isOrientationEqual(this->pose.p.orientation,a.pose.p.orientation)));
}

void Node::print()
{
  std::cout<<"\n Node X="<<pose.p.position.x<<" Y="<<pose.p.position.y<<" Z="<<pose.p.position.z<<" F value="<<f_value;
}

}
