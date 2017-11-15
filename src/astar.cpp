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
#include "sspp/astar.h"

namespace SSPP
{

Astar::Astar(ros::NodeHandle & n, Robot *rob, int progressDisplayFrequency):
    nh(n),
    robot(rob),
    root(NULL),
    p(NULL),
    openList(NULL),
    closedList(NULL),
    globalcount(0),
    debug(false),
    progressDisplayFrequency(progressDisplayFrequency),
    debugDelay(0)
{    
    childPosePub         = nh.advertise<geometry_msgs::PoseArray>("child_pose",10);
    childSensorsPub      = nh.advertise<geometry_msgs::PoseArray>("child_sensors",10);
    parentPosePub        = nh.advertise<geometry_msgs::PoseArray>("parent_pose",10);
    parenSensorsPub      = nh.advertise<geometry_msgs::PoseArray>("parent_sensors",10);
    branchPub            = nh.advertise<visualization_msgs::Marker>("branch",10);
    octomapChildPub      = nh.advertise<octomap_msgs::Octomap>("octomap_child", 10);

    nodeToBeVisNum       = 0;//if 0 then it will continue planning till the target otherwise it will stop planning on the specified viewpoint number and will visualize the steps of choosing next viewpoint
    nodesCounter         = 0;
}

Astar::Astar():
    heuristic(NULL),
    root(NULL),
    p(NULL),
    openList(NULL),
    closedList(NULL),
    globalcount(0),
    debug(true)
{
}

Astar::~Astar()
{
    if(openList)
    {
        openList->free();
        delete openList;
    }
    if(closedList)
    {
        closedList->free();
        delete closedList;
    }
}

// find the nearest node to the start
void Astar::findRoot() throw (SSPPException)
{
    SearchSpaceNode * temp;
    if(!this->searchspace)
    {
        throw(SSPPException((char*)"No SearchSpace Defined"));
        return;
    }
    double distance,shortestDist = std::numeric_limits<double>::max();
    // allocate and setup the root node
    root = new Node;
    temp = this->searchspace;
    while(temp!=NULL)
    {
        distance = Dist(temp->location,start.p);
        // Initialize the root node information and put it in the open list
        if (distance < shortestDist)
        {
            shortestDist = distance;

            root->pose.p.position.x = temp->location.position.x;
            root->pose.p.position.y = temp->location.position.y;
            root->pose.p.position.z = temp->location.position.z;
            root->pose.p.orientation.x = temp->location.orientation.x;
            root->pose.p.orientation.y = temp->location.orientation.y;
            root->pose.p.orientation.z = temp->location.orientation.z;
            root->pose.p.orientation.w = temp->location.orientation.w;
            root->senPoses.erase(root->senPoses.begin(),root->senPoses.end());
            for(int i=0; i<temp->sensorLocation.poses.size();i++)
            {
                Pose tempPose;
                tempPose.p.position.x = temp->sensorLocation.poses[i].position.x;
                tempPose.p.position.y = temp->sensorLocation.poses[i].position.y;
                tempPose.p.position.z = temp->sensorLocation.poses[i].position.z;
                tempPose.p.orientation.x = temp->sensorLocation.poses[i].orientation.x;
                tempPose.p.orientation.y = temp->sensorLocation.poses[i].orientation.y;
                tempPose.p.orientation.z = temp->sensorLocation.poses[i].orientation.z;
                tempPose.p.orientation.w = temp->sensorLocation.poses[i].orientation.w;

                root->senPoses.push_back(tempPose);

            }
            root->id = temp->id;
        }
        temp = temp->next;
    }

    root->id = 0;
    root->parent = NULL;
    root->next = NULL;
    root->prev = NULL;
    root->g_value = 0;
    root->distance = 0;
    root->coverage = 0;
    root->h_value = 0;
    heuristic->calculateHeuristic(root);
    root->depth = 0;
    //Translate(root->pose,start.phi);
    std::cout<<"\n"<<"	---->>>Root is Set to be X="<<root->pose.p.position.x<<" Y="<<root->pose.p.position.y<<" Z="<<root->pose.p.position.z;
}


void Astar::setRobot(Robot*rob)
{
    this->robot = rob;
}

void Astar::setHeuristicFucntion(Heuristic* heuristicFun)
{
    heuristic = heuristicFun;
}

//TODO: Randa call this inside the AStart Search with the appropriate parameters
void Astar::displayOctree()
{
  /*
  //display the child and the octree
  if(nodesCounter == nodeToBeVisNum && nodeToBeVisNum != 0)
  {
    geometry_msgs::Pose parent;

    geometry_msgs::Pose child;
    geometry_msgs::Point linePoint;

    std::cout<<"I entered tree childrens of the desired node "<<std::endl;

    child = curChild->pose.p;
    parent = current->pose.p;
    childPose.poses.push_back(child);
    parentPose.poses.push_back(parent);
    for(int i=0; i<curChild->senPoses.size();i++)
    {
      childSensors.poses.push_back(curChild->senPoses[i].p);
      parentSensors.poses.push_back(current->senPoses[i].p);
    }

    linePoint.x = current->pose.p.position.x;
    linePoint.y = current->pose.p.position.y;
    linePoint.z = current->pose.p.position.z;
    lineSegments.push_back(linePoint);
    linePoint.x = child.position.x;
    linePoint.y = child.position.y;
    linePoint.z = child.position.z;
    lineSegments.push_back(linePoint);

    //visualization
    childPose.header.frame_id= "map";
    childPose.header.stamp = ros::Time::now();
    childPosePub.publish(childPose);
    childSensors.header.frame_id= "map";
    childSensors.header.stamp = ros::Time::now();
    childSensorsPub.publish(childSensors);
    parentPose.header.frame_id= "map";
    parentPose.header.stamp = ros::Time::now();
    parentPosePub.publish(parentPose);
    parentSensors.header.frame_id= "map";
    parentSensors.header.stamp = ros::Time::now();
    parenSensorsPub.publish(parentSensors);
    visualization_msgs::Marker linesList1 = drawLines(lineSegments,1,6,100000,0.1);
    branchPub.publish(linesList1);


    octomap_msgs::Octomap octomap ;
    octomap.binary = 1 ;
    octomap.id = 1 ;
    octomap.resolution =0.25;
    octomap.header.frame_id = "map";
    octomap.header.stamp = ros::Time::now();
    bool res = octomap_msgs::fullMapToMsg(*curChild->octree, octomap);
    if(res)
    {
      octomapChildPub.publish(octomap);
    }
    else
    {
      ROS_WARN("OCT Map serialization failed!");
    }
    childSensors.poses.erase(childSensors.poses.begin(),childSensors.poses.end() );
    childPose.poses.erase(childPose.poses.begin(),childPose.poses.end() );
    lineSegments.erase(lineSegments.begin(), lineSegments.end());
    ros::Duration(2).sleep();
  }
  */
}

Node *Astar::astarSearch(Pose start)
{
    Node *path = NULL;
    int NodesExpanded = 0;
    globalID = 0;
    bool condition;
    if(this->tree.size() > 0)
        this->tree.clear();
    if(!openList)
    {
        openList   = new LList;
    }
    if(!closedList)
    {
        closedList = new LList;
    }
    // Be sure that open and closed lists are empty
    openList->free();
    closedList->free();
    if(!this->searchspace)
    {
        std::cout<<"\nGenerate SearchSpace before Searching !!!";
        return NULL;
    }

    this->start.p.position.x = start.p.position.x;
    this->start.p.position.y = start.p.position.y;
    this->start.p.position.z = start.p.position.z;
    this->start.p.orientation.x = start.p.orientation.x;
    this->start.p.orientation.y = start.p.orientation.y;
    this->start.p.orientation.z = start.p.orientation.z;
    this->start.p.orientation.w = start.p.orientation.w;

    std::cout<<"\n	--->>> Search Started <<<---"<<std::endl;
    findRoot();

    openList->add(root,heuristic->isCost());
    // while openList is not empty
    int count = 0;
    while(openList->Start != NULL)
    {
        if(progressDisplayFrequency > 0 && (count++%progressDisplayFrequency) == 0)
        {
            heuristic->displayProgress(tree);
            //seconds to usec
            if(debugDelay!=0)
                usleep(debugDelay*1000000);
        }

        // Get the node with the highest cost (first node) (it was the cheapest one before since we were taking the lower cost but now it is converted to a reward function)
        current = new Node(openList->getHead());
        openList->remove(openList->getHead());

        // put the current node onto the closed list, ==>> already visited List
        closedList->add(current,heuristic->isCost());

        NodesExpanded++;

        if ((heuristic->terminateConditionReached(current) && current!= root) || (nodesCounter==nodeToBeVisNum && nodeToBeVisNum!=0))
        {
            //the last node in the path
            current->next = NULL;
            std::cout<<"*************Cumulative distance : "<<current->distance<<"************ \n";
            std::cout<<"\n"<<"	--->>> Goal state reached with :"<<globalID<< "nodes created and :"<<NodesExpanded<<" nodes expanded <<<---";
            fflush(stdout);
            p = current;
            path = NULL;
            while (p != NULL)
            {
                if(p->prev != NULL)
                    (p->prev)->next = p->next;
                if(p->next != NULL)
                    (p->next)->prev = p->prev;
                // check if we're removing the top of the list
                if(p == closedList->Start)
                    closedList->next();
                // set it up in the path
                p->next = path;
                path = p;
                p = p->parent;
            }
            // now delete all nodes on OPEN and Closed Lists
            openList->free();
            closedList->free();
            return path;
        }
        debug = false;
        // Create List of Children for the current NODE
        if(!(childList = makeChildrenNodes(current)))
        {
            std::cout<<"\n	--->>> Search Ended On this Branch / We Reached a DEAD END <<<---";
        }
        // insert the children into the OPEN list according to their f values
        nodesCounter++;

        while (childList != NULL)
        {
            curChild  = childList;
            childList = childList->next;
            curChild->next = NULL;
            curChild->prev = NULL;
            // calculate f_value
            heuristic->calculateHeuristic(curChild);
            //TODO:: pass the correct parameters to the display function
            displayOctree();
            globalcount++;
            Node * p= NULL;
            if(debug)
            {
                std::cout<<"\n ID= "<<curChild->id<<" global count:"<<globalcount<<" nodeExpanded:"<<NodesExpanded<<" mem address:"<<curChild;
                std::cout<<"\ncurChildren f value= "<<curChild->f_value;
                std::cout<<"\nFinding the curchild : "<<curChild->pose.p.position.x<<" "<< curChild->pose.p.position.y<<" "<<curChild->pose.p.position.z<<" "<<curChild->pose.p.orientation.x<<" "<<curChild->pose.p.orientation.y<<" "<<curChild->pose.p.orientation.z<<" "<<curChild->pose.p.orientation.w;
            }
            if(heuristic->isCost())
                condition = (p->f_value <=curChild->f_value);
            else
                condition = (p->f_value >=curChild->f_value);
            if(openList!=NULL)
            {
              p = openList->find(curChild);
              if(p)
              {
                if(condition)
                {
                  freeNode(curChild);
                  curChild = NULL;
                  continue;
                }
              }
            }
            if(closedList!=NULL)
            {
              if((p = closedList->find(curChild)))
              {
                if(condition)
                {
                  freeNode(curChild);
                  curChild = NULL;
                  continue;
                }
              }
            }
            // remove any similar node in open list as it's higher cost
            openList->remove(curChild);
            // add the new one, it's lower cost
            openList->add(curChild,heuristic->isCost());
        }
        // Test to see if we have expanded too many nodes without a solution
        if (current->id > this->MAXNODES)
        {
            std::cout<<"\nExpanded:"<<current->id<<" Nodes which is more than the maximum allowed MAXNODE:"<<MAXNODES;
            //Delete Nodes in Open and Closed Lists
            closedList->free();
            openList->free();
            std::cout<<"\nThe closed list and open list have been cleared"<<"\n";fflush(stdout);
            path = NULL;
            return path; // Expanded more than the maximium nodes state
        }
    }
    closedList->free();
    std::cout<<"\n	--->>>No Path Found<<<---";
    return NULL;
}

Node *Astar::makeChildrenNodes(Node *parent)
{
    geometry_msgs::Pose P;
    Node  *p, *q, *r;
    SearchSpaceNode *temp;

    P.position.x     = parent->pose.p.position.x;
    P.position.y     = parent->pose.p.position.y;
    P.position.z     = parent->pose.p.position.z;
    P.orientation.x  = parent->pose.p.orientation.x;
    P.orientation.y  = parent->pose.p.orientation.y;
    P.orientation.z  = parent->pose.p.orientation.z;
    P.orientation.w  = parent->pose.p.orientation.w;

    Tree t;
    t.location = P;
    if(!searchspace)
        return NULL;
    temp = searchspace;

    // Locate the node in the Search Space, necessary to determine the neighbours
    while(temp!=NULL)
    {
        //added the orientation since we have different
        if(isPositionEqual(temp->location.position,P.position) && isOrientationEqual(temp->location.orientation,P.orientation))
            break;
        temp = temp->next;
    }

    if(!temp)
    {
        return NULL;
    }
    q = NULL;

    if(debug)
    {
        std::cout<<"\n\n\n#############children Size: "<< temp->children.size() <<" ##################\n\n\n";fflush(stdout);
    }

    // Check Each neighbour
    for(int i=0;i<temp->children.size();i++)
    {
        //TODO: check for collision before adding the node, use previous implementation for reference
        p = new Node;
        p->pose.p.position.x = temp->children[i]->location.position.x;
        p->pose.p.position.y = temp->children[i]->location.position.y;
        p->pose.p.position.z = temp->children[i]->location.position.z;
        p->pose.p.orientation.x = temp->children[i]->location.orientation.x;
        p->pose.p.orientation.y = temp->children[i]->location.orientation.y;
        p->pose.p.orientation.z = temp->children[i]->location.orientation.z;
        p->pose.p.orientation.w = temp->children[i]->location.orientation.w;

        for(int j=0; j<temp->children[i]->sensorLocation.poses.size();j++)
        {
            Pose tempPose;
            tempPose.p.position.x = temp->children[i]->sensorLocation.poses[j].position.x;
            tempPose.p.position.y = temp->children[i]->sensorLocation.poses[j].position.y;
            tempPose.p.position.z = temp->children[i]->sensorLocation.poses[j].position.z;
            tempPose.p.orientation.x = temp->children[i]->sensorLocation.poses[j].orientation.x;
            tempPose.p.orientation.y = temp->children[i]->sensorLocation.poses[j].orientation.y;
            tempPose.p.orientation.z = temp->children[i]->sensorLocation.poses[j].orientation.z;
            tempPose.p.orientation.w = temp->children[i]->sensorLocation.poses[j].orientation.w;

            p->senPoses.push_back(tempPose);
        }
        // move the " in closed list detection" to the astar
        p->id = temp->children[i]->id;
        p->parent = parent;
        p->next   = q;
        p->prev   = NULL;
        p->depth  = parent->depth + 1;
        p->id     = globalID++;
        q = p;
        t.children.push_back(p->pose.p);
    }
    // Save the search tree so that it can be displayed later
    if (t.children.size() > 0)
        tree.push_back(t);
    if(debug)
    {
        std::cout<<" Making children nodes: "<<t.children.size()<<"\n"; fflush(stdout);
    }
    return q;
}

// Free node function
void Astar::freeNode(Node *n)
{
    delete n;
}

void Astar::setProgressDisplayFrequency(int progressDisplayFrequency)
{
    this->progressDisplayFrequency = progressDisplayFrequency;
}

void Astar::setDebugDelay(double delay)
{
    this->debugDelay = delay;
}

}
