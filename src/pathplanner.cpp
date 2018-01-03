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
#include "sspp/pathplanner.h"
namespace SSPP
{

PathPlanner::PathPlanner(ros::NodeHandle &nh, Robot *rob, double regGridConRadius, int progressDisplayFrequency,std::vector<Sensors> &rSensors):
    nh(nh),
    Astar(nh,rob,progressDisplayFrequency),
    regGridConRadius(regGridConRadius),
    sampleOrientations(false),
    samplesFiltering(false),
    multiAgentSupport(false),
    insertSearchSpace(true),
    robotSensors(rSensors)
{

    this->MAXNODES = 0;
}

PathPlanner::PathPlanner(ros::NodeHandle &nh, Robot *rob, double regGridConRadius, int progressDisplayFrequency):
    nh(nh),
    Astar(nh,rob,progressDisplayFrequency),
    regGridConRadius(regGridConRadius),
    sampleOrientations(false),
    samplesFiltering(false),
    multiAgentSupport(false),
    insertSearchSpace(true)
{

}

PathPlanner::~PathPlanner()
{
    freeResources();
    std::cout<<"\n	--->>> Allocated Memory FREED <<<---"<<std::endl;
}

void PathPlanner::freeResources()
{
    freeSearchSpace();
    freePath();
    p=root=NULL;
}

void PathPlanner::freePath()
{
    for (std::vector<Node*>::iterator it = paths.begin() ; it != paths.end(); ++it)
    {
        Node * path = *it;
        while(path != NULL)
        {
            p = path->next;
            delete path;
            path = p;
        }
    }
    paths.clear();
}

void PathPlanner::setConRad(double a)
{
    regGridConRadius = a;
}

void PathPlanner::setMultiAgentSupport(bool allowMultiAgentSupport)
{
    this->multiAgentSupport = allowMultiAgentSupport;
}

void PathPlanner::generateRegularGrid(geometry_msgs::Pose gridStartPose,geometry_msgs::Vector3 gridSize, float gridRes, bool sampleOrientations, float orientationRes, bool samplesFiltering, bool insertSearchSpace)
{
    this->gridResolution = gridRes;
    this->sampleOrientations = sampleOrientations;
    this->orientationResolution = orientationRes;
    this->samplesFiltering = samplesFiltering;
    this->insertSearchSpace = insertSearchSpace;
    generateRegularGrid(gridStartPose,gridSize);
}

void PathPlanner::generateRegularGrid(geometry_msgs::Pose gridStartPose,geometry_msgs::Vector3 gridSize, float gridRes)
{
    this->gridResolution = gridRes;
    generateRegularGrid(gridStartPose,gridSize);
}

void PathPlanner::generateRegularGrid(geometry_msgs::Pose gridStartPose, geometry_msgs::Vector3 gridSize)
{
    geometry_msgs::Pose pose, sensorLoc;
    geometry_msgs::PoseArray correspondingSensorPose;
    for(int j=0; j<robotSensors.size();j++)
    {
        sensorsFilteredPoses.push_back(correspondingSensorPose);
    }
    int orientationsNum = 1;
    if(sampleOrientations)
    {
        orientationsNum= 360.0f/orientationResolution;
    }
    std::cout<<"\nNumber of Orientation Samples:"<<orientationsNum<<" sample orientations?"<<sampleOrientations<<" orientation sampling resolution:"<<orientationResolution; fflush(stdout);
    int numSamples = 0;

    for(double z = (gridStartPose.position.z - gridSize.z/2.0); z<=(gridStartPose.position.z + gridSize.z/2.0); z+=gridResolution)
    {
      for(double x = (gridStartPose.position.x - gridSize.x/2.0); x<=(gridStartPose.position.x + gridSize.x/2.0); x+=gridResolution)
      {
        for(double y = (gridStartPose.position.y - gridSize.y/2.0); y<=(gridStartPose.position.y + gridSize.y/2.0); y+=gridResolution)
        {
          pose.position.z=z;
          pose.position.y=y;
          pose.position.x=x;

          if(sampleOrientations)
          {
            // in radians
            double yaw=0.0;
            tf::Quaternion tf ;
            for(int i=0; i<orientationsNum;i++)
            {
              tf = tf::createQuaternionFromYaw(yaw);
              pose.orientation.x = tf.getX();
              pose.orientation.y = tf.getY();
              pose.orientation.z = tf.getZ();
              pose.orientation.w = tf.getW();
              yaw+=(orientationResolution*M_PI/180.0f);
              for(int j=0; j<robotSensors.size();j++)
              {
                sensorLoc = robotSensors[j].robot2sensorTransformation(pose);
                correspondingSensorPose.poses.push_back(sensorLoc);
              }

              if(samplesFiltering)
              {
                //Note: the accuracy threshhold defines the maximum error that you want to get, any waypoint (with its viewpoints) that extracts point cloud with bigger error than this threshold
                // will be added to the accuracy clusters to be re-sampled with lower resolution (results with nearer waypoints which provide better accuracy & low error)

                //Note: if you chose a very small depth, the process of filtering will take time since the accuracy clusters will increase so much and as a result, the number of waypoints will increase alot!!
                // it is also affected by the decrement step of the dynamic sampling
                double desiredMaxDepth = 6.9; //meters
                double maxErrorThresh = 0.0000285 * desiredMaxDepth * desiredMaxDepth; //meters squared (this equation with the constant 0.0000285 is taken from a paper that studied the accuracy of kinect sensor  )
                if(heuristic->isFilteringConditionSatisfied(pose, correspondingSensorPose, 1, 4, globalCloud, accuracyClusters,maxErrorThresh))
                {
                  if(insertSearchSpace)
                  {
                    insertNode(pose,correspondingSensorPose);
                    robotFilteredPoses.poses.push_back(pose);

                    for(int j=0; j<robotSensors.size();j++)
                    {
                      sensorsFilteredPoses[j].poses.push_back(correspondingSensorPose.poses[j]);
                    }
                  }
                  numSamples++;
                  //                                std::cout<<"number of samples "<<numSamples<<std::endl;
                }
              }
              else
              {
                if(insertSearchSpace)
                  insertNode(pose,correspondingSensorPose);
              }
              correspondingSensorPose.poses.erase(correspondingSensorPose.poses.begin(),correspondingSensorPose.poses.end());
            }
          }
          else
          {
            pose.orientation.x=0;pose.orientation.y=0;pose.orientation.z=0;pose.orientation.w=1;
            for(int j=0; j<robotSensors.size();j++)
            {
              sensorLoc = robotSensors[j].robot2sensorTransformation(pose);
              correspondingSensorPose.poses.push_back(sensorLoc);
            }
            if(insertSearchSpace)
              insertNode(pose,correspondingSensorPose);
            correspondingSensorPose.poses.erase(correspondingSensorPose.poses.begin(),correspondingSensorPose.poses.end());
            numSamples++;
          }

        }
      }
    }
    std::cout<<"\n	--->>> REGULAR GRID GENERATED SUCCESSFULLY <<<--- Samples:"<<numSamples++; fflush(stdout);
}


void PathPlanner::loadRegularGrid(const char *filename1, const char *filename2, const char *filename3)
{
    geometry_msgs::Pose pose, sensorPose;
    geometry_msgs::PoseArray correspondingSensorPose;
    double locationx,locationy,locationz,qx,qy,qz,qw;
    double senLocx,senLocy,senLocz,senqx,senqy,senqz,senqw;

    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");

    assert(filename2 != NULL);
    filename2 = strdup(filename2);
    FILE *file2 = fopen(filename2, "r");

    assert(filename3 != NULL);
    filename3 = strdup(filename3);
    FILE *file3 = fopen(filename3, "r");
    if (!file1 || !file2 || !file3)
    {
        std::cout<<"\nCan not open Search Space File";
        fclose(file1);
        fclose(file2);
        fclose(file3);
    }
    //asuming using one sensor
    while (!feof(file1) && !feof(file2) && !feof(file3))
    {
        fscanf(file1,"%lf %lf %lf %lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qx,&qy,&qz,&qw);
        pose.position.x = locationx;
        pose.position.y = locationy;
        pose.position.z = locationz;
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;

        fscanf(file2,"%lf %lf %lf %lf %lf %lf %lf\n",&senLocx,&senLocy,&senLocz,&senqx,&senqy,&senqz,&senqw);
        sensorPose.position.x = senLocx;
        sensorPose.position.y = senLocy;
        sensorPose.position.z = senLocz;
        sensorPose.orientation.x = senqx;
        sensorPose.orientation.y = senqy;
        sensorPose.orientation.z = senqz;
        sensorPose.orientation.w = senqw;
        correspondingSensorPose.poses.push_back(sensorPose);


        fscanf(file3,"%lf %lf %lf %lf %lf %lf %lf\n",&senLocx,&senLocy,&senLocz,&senqx,&senqy,&senqz,&senqw);
        sensorPose.position.x = senLocx;
        sensorPose.position.y = senLocy;
        sensorPose.position.z = senLocz;
        sensorPose.orientation.x = senqx;
        sensorPose.orientation.y = senqy;
        sensorPose.orientation.z = senqz;
        sensorPose.orientation.w = senqw;
        correspondingSensorPose.poses.push_back(sensorPose);

        insertNode(pose,correspondingSensorPose);
        correspondingSensorPose.poses.erase(correspondingSensorPose.poses.begin(),correspondingSensorPose.poses.end());
    }
    fclose(file1);
    fclose(file2);
    fclose(file3);
    std::cout<<"\n	--->>> REGULAR GRID GENERATED SUCCESSFULLY <<<---	";
}

std::vector<geometry_msgs::Point> PathPlanner::getSearchSpace()
{
    SearchSpaceNode *temp = searchspace;
    std::vector<geometry_msgs::Point> pts;
    while (temp != NULL)
    {
        geometry_msgs::Point pt;
        pt.x= temp->location.position.x;
        pt.y= temp->location.position.y;
        pt.z= temp->location.position.z;
        pts.push_back(pt);
        temp = temp->next;
    }
    return pts;
}

//gets the robot waypoints and the sensors viewpoints each in seperate vector
void PathPlanner::getRobotSensorPoses(geometry_msgs::PoseArray& robotPoses, std::vector<geometry_msgs::PoseArray> &sensorsPoses)
{
    SearchSpaceNode *temp = searchspace;

    for(int i = 0 ; i<temp->sensorLocation.poses.size(); i++)
    {
        geometry_msgs::PoseArray sensor;
        sensorsPoses.push_back(sensor);
    }
    while (temp != NULL)
    {
        geometry_msgs::Pose pose;
        pose.position.x= temp->location.position.x;
        pose.position.y= temp->location.position.y;
        pose.position.z= temp->location.position.z;
        pose.orientation.x= temp->location.orientation.x;
        pose.orientation.y= temp->location.orientation.y;
        pose.orientation.z= temp->location.orientation.z;
        pose.orientation.w= temp->location.orientation.w;
        robotPoses.poses.push_back(pose);
        for(int i=0; i<temp->sensorLocation.poses.size();i++)
        {
            sensorsPoses[i].poses.push_back(temp->sensorLocation.poses[i]);
        }
        temp = temp->next;
    }
}

//gets the robot waypoints and the sensors viewpoints in one vector
void PathPlanner::getRobotSensorPoses(geometry_msgs::PoseArray& robotPoses, geometry_msgs::PoseArray& sensorPoses)
{
    SearchSpaceNode *temp = searchspace;
    while (temp != NULL)
    {
        geometry_msgs::Pose pose;
        pose.position.x= temp->location.position.x;
        pose.position.y= temp->location.position.y;
        pose.position.z= temp->location.position.z;
        pose.orientation.x= temp->location.orientation.x;
        pose.orientation.y= temp->location.orientation.y;
        pose.orientation.z= temp->location.orientation.z;
        pose.orientation.w= temp->location.orientation.w;
        robotPoses.poses.push_back(pose);
        for(int i=0; i<temp->sensorLocation.poses.size();i++)
        {
            sensorPoses.poses.push_back(temp->sensorLocation.poses[i]);
        }
        temp = temp->next;
    }
}

void PathPlanner::printNodeList()
{
    int pathIndex = 0;
    for (std::vector<Node*>::iterator it = paths.begin() ; it != paths.end(); ++it)
    {
        if(!(p = *it))
            return ;
        std::cout<<"\nDisplaying Path["<<++pathIndex<<"]\n";
        printPath(p);
    }
}

void PathPlanner::printPath(Node *path)
{
    Node * p = path;
    geometry_msgs::Pose pixel;
    std::cout<<"\n--------------------   START OF LIST ----------------------";
    int step = 1;
    while(p !=NULL)
    {
        pixel.position.x =  p->pose.p.position.x;
        pixel.position.y =  p->pose.p.position.y;
        pixel.position.z =  p->pose.p.position.z;

        std::cout<<"\nStep [" << step++ <<"] x["<< pixel.position.x<<"] y["<<pixel.position.y<<"] z["<<pixel.position.z<< "]";
        std::cout<<"\tG cost="<<p->g_value<<"\tH cost="<<p->h_value<<"\tFcost="<<p->f_value;
        if (p->next !=NULL)
        {
            //cout<<"\tNext Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x));
            //cout<<"\tAngle Diff ="<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(anglediff(p->angle,atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x)));
        }
        p = p->next;
    }
    std::cout<<"\n--------------------   END OF LIST ---------------------- ";
}

void PathPlanner::printPath(int index)
{
    if(index>=0 && index<paths.size())
        printPath(paths.at(index));
    else
        std::cout<<"\n Index out of bound";
}

void PathPlanner::printLastPath()
{
    if(paths.size()>0)
    {
        printPath(paths[paths.size()-1]);
    }
    else
        std::cout<<"\n No Paths to print";
}

void PathPlanner::disconnectNodes()
{
    SearchSpaceNode * ss = searchspace;
    while (ss!=NULL)
    {
        ss->children.clear();
        ss = ss->next;
    }
}

void PathPlanner::blockPath(Node *path)
{
    while(path!=NULL)
    {
        removeNode(path->pose.p);
        path = path->next;
    }
}

Node *PathPlanner::startSearch(Pose startPose)
{
    if(multiAgentSupport)
    {
        disconnectNodes();
        for(std::vector<Node*>::iterator it = paths.begin(); it!=paths.end();it++)
        {
            blockPath(*it);
        }
        connectNodes();
    }
    Node *p = astarSearch(startPose);
    if(multiAgentSupport)
    {
        paths.push_back(p);
    }
    else
    {
        //only one at any time
        freePath();
        paths.push_back(p);
    }
    return p;
}

void PathPlanner::connectNodes()
{
    SearchSpaceNode * S;
    SearchSpaceNode *temp;
    double distance;
    if (!searchspace)
        return;
    temp = searchspace;
    int numConnections=0;
    while (temp!=NULL)
    {
        S = searchspace;
        while (S!=NULL)
        {
            distance = Dist(S->location,temp->location);
            if (distance <= regGridConRadius && S != temp)// && distance !=0)
            {
                //check if parent and child are in the same position
                if (S->location.position.x != temp->location.position.x || S->location.position.y != temp->location.position.y || S->location.position.z != temp->location.position.z )
                {
                    if (heuristic->isConnectionConditionSatisfied(temp,S))
                    {
                        temp->children.push_back(S);
                        numConnections++;
                    }
                }
                //child and parent are in the same position.
                // TODO: check this logic
                else
                {
                    numConnections++;
                    temp->children.push_back(S);
                }
            }
            S = S->next;
        }
        temp = temp->next;
    }
    std::cout<<"\n	--->>> NODES CONNECTED <<<---	Total number of connections:"<<numConnections;
    this->MAXNODES = numConnections;//searchspace->id;
}

void PathPlanner::connectClustersInternalNodes(SearchSpaceNode * space, double connRadius)
{
    SearchSpaceNode * S;
    SearchSpaceNode *temp;

    double distance;
    if (!space)
        return;

    temp=space;
    int numConnections=0;
    while (temp!=NULL)
    {
        S = space;
        while (S!=NULL)
        {
            distance = Dist(S->location,temp->location);
            if (distance <= connRadius && S != temp)// && distance !=0)
            {
                //check if parent and child are in the same position
                if (S->location.position.x != temp->location.position.x || S->location.position.y != temp->location.position.y || S->location.position.z != temp->location.position.z )
                {
                    //remeber: used nodeExists(temp2->location) & nodeExists(S->location) since we need to connect nodes in search space not in the temp space
                    // when we used temp space and pushed as child, the connection will be affected later if the node is deleted (CHECK)
                    if (heuristic->isConnectionConditionSatisfied(temp,S))
                    {
                        numConnections++;
                        nodeExists(temp->location)->children.push_back(nodeExists(S->location));//to put the connection in the searchspace that will be used in the path planning
                    }
                }
                //child and parent are in the same position.
                else
                {
                    numConnections++;
                    nodeExists(temp->location)->children.push_back(nodeExists(S->location));//to put the connection in the searchspace that will be used in the path planning
                }

            }
            S = S->next;
        }
        temp = temp->next;
    }
    std::cout<<"\n	--->>> INERNAL NODES CONNECTED <<<---	Total number of connections:"<<numConnections;
    this->MAXNODES += numConnections;//searchspace->id;
}

void PathPlanner::connectToNN(pcl::PointCloud<pcl::PointXYZ> cloudHull1, pcl::PointCloud<pcl::PointXYZ> cloudHull2)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHullPtr1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHullPtr2 (new pcl::PointCloud<pcl::PointXYZ>);

    cloudHullPtr1->points = cloudHull1.points;
    cloudHullPtr2->points = cloudHull2.points;
    //2 is used here representing the number of cloud used to connect
    int numConnections=0;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloudHullPtr1);


    for(int i =0; i<cloudHullPtr2->size() ; i++)//looping through the hull
    {

        pcl::PointXYZ searchPoint = cloudHullPtr2->points[i];

        //remember: created vectors of size 1 since the nearest one is needed, and KDTree sort nearest points based on the distance from the searchPoint
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);


        if ( kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {

            pcl::PointXYZ nearestPoint = cloudHullPtr1->points[ pointIdxNKNSearch[0] ];

            // in order to connet , we have to create nodes with pose and orientation
            //( 8 orientaitons since we lost the orientation information when we generated the hull)
            SearchSpaceNode* nearestnode = new SearchSpaceNode;
            nearestnode->location.position.x   = nearestPoint.x;
            nearestnode->location.position.y   = nearestPoint.y;
            nearestnode->location.position.z   = nearestPoint.z;

            SearchSpaceNode* node = new SearchSpaceNode;
            node->location.position.x   = searchPoint.x;
            node->location.position.y   = searchPoint.y;
            node->location.position.z   = searchPoint.z;

            int orientationsNum= 360.0f/orientationResolution;
            // in radians
            double yaw1=0.0;
            tf::Quaternion tf ;
            for(int k=0; k<orientationsNum;k++)
            {
                tf = tf::createQuaternionFromYaw(yaw1);
                node->location.orientation.x  = tf.getX();
                node->location.orientation.y  = tf.getY();
                node->location.orientation.z  = tf.getZ();
                node->location.orientation.w  = tf.getW();
                double yaw2=0.0;
                for(int i=0; i<orientationsNum;i++)
                {
                    tf = tf::createQuaternionFromYaw(yaw2);
                    nearestnode->location.orientation.x  = tf.getX();
                    nearestnode->location.orientation.y  = tf.getY();
                    nearestnode->location.orientation.z  = tf.getZ();
                    nearestnode->location.orientation.w  = tf.getW();

                    yaw2+=(orientationResolution*M_PI/180.0f);

                    if (node != nearestnode)
                    {
                        //check if parent and child are in the same position
                        if (node->location.position.x != nearestnode->location.position.x || node->location.position.y != nearestnode->location.position.y || node->location.position.z != nearestnode->location.position.z )
                        {
                            //remeber: used nodeExists(temp2->location) & nodeExists(S->location) since we need to connect nodes in search space not in the temp space
                            if (heuristic->isConnectionConditionSatisfied(node,nearestnode))
                            {
                                SearchSpaceNode* nodeTemp = nodeExists(node->location);
                                SearchSpaceNode* nearestTemp = nodeExists(nearestnode->location);
                                if(nodeTemp!=NULL && nearestTemp!=NULL)//it exists in the search space
                                {
                                    numConnections++;
                                    nodeTemp->children.push_back(nearestTemp);//to put the connection in the searchspace that will be used in the path planning
                                    numConnections++;
                                    nearestTemp->children.push_back(nodeTemp);//to put the connection in the searchspace that will be used in the path planning
                                }

                            }
                        }
                        //child and parent are in the same position.
                        else
                        {
                            SearchSpaceNode* nodeTemp = nodeExists(node->location);
                            SearchSpaceNode* nearestTemp = nodeExists(nearestnode->location);
                            if(nodeTemp!=NULL && nearestTemp!=NULL)//it exists in the search space
                            {
                                numConnections++;
                                nodeTemp->children.push_back(nearestTemp);//to put the connection in the searchspace that will be used in the path planning
                                numConnections++;
                                nearestTemp->children.push_back(nodeTemp);//to put the connection in the searchspace that will be used in the path planning
                            }
                        }

                    }
                }
                yaw1+=(orientationResolution*M_PI/180.0f);

            }//end of orientaiton loop

        }//end of if statement

    }//end of looping through hull

    std::cout<<"\n	--->>> CLUSTER WITH NEIGBOURS NODES CONNECTED <<<---	Total number of connections:"<<numConnections;
    this->MAXNODES += numConnections;//searchspace->id;
}

void PathPlanner::dynamicNodesGenerationAndConnection(geometry_msgs::Pose gridStartPose, geometry_msgs::Vector3 gridSize, double startRes, double resDecrement)
{

    //TODO::Change this LOGIC !!!!
    octomap::OcTree oct(0.1);
    //discretize & filtering
    double res = startRes;
    this->generateRegularGrid(gridStartPose, gridSize,startRes,true,45,true,true);
    SearchSpaceNode * S =insertTempSearchSpace(robotFilteredPoses,sensorsFilteredPoses);

    //connect internally
    double connRadius = std::sqrt((res*res) + (res*res)) + 0.01;
    this->connectClustersInternalNodes(S,connRadius);
    freeTempSearchSpace(S);

    //find the outer points (convexs hull)
    pcl::PointCloud<pcl::PointXYZ> initialHullCloud;
    //heuristic->findClusterOuterPoints(robotFilteredPoses,initialHullCloud); //not accurate, it generate outer points with precession problem,
    //  I converted the samples to pcl cloud since KD Tree take point cloud
    pcl::PointCloud<pcl::PointXYZ> initialPositionsCloud;
    for(int i =0; i<robotFilteredPoses.poses.size();i++)
    {
        pcl::PointXYZ pt;
        pt.x = robotFilteredPoses.poses[i].position.x;
        pt.y = robotFilteredPoses.poses[i].position.y;
        pt.z = robotFilteredPoses.poses[i].position.z;
        initialPositionsCloud.push_back(pt);
    }
    initialHullCloud += initialPositionsCloud;
    robotFilteredPoses.poses.erase(robotFilteredPoses.poses.begin(), robotFilteredPoses.poses.end());
    sensorsFilteredPoses.erase(sensorsFilteredPoses.begin(), sensorsFilteredPoses.end());

    //dynamic sampling loop
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    int j=0;
    pcl::PointCloud<pcl::PointXYZ> hullCloud1;
    hullCloud1.points = initialHullCloud.points;

    while(true){
        ////////////////////////////find the Uncovered part////////////////////////////
        globalCloudPtr->points=globalCloud.points;
        pcl::PointCloud<pcl::PointXYZ>::Ptr diffPtr (new pcl::PointCloud<pcl::PointXYZ>);
        double uncoveredPercent = heuristic->pointCloudDiff(globalCloudPtr,diffPtr);
        std::cout<<"points difference: "<< uncoveredPercent <<std::endl;
        ///////////check the difference percentage///////////////
        //it is hard to reach full coverage for some structures (ex. aircraft) feasible samples that doesn't go under z=0 are selected and also
        // the samples filtering distance affects this part
        if(uncoveredPercent<=0.01) //termination condition
            break;

        ///////////cluster the uncovered part into regions///////////////
        std::vector<pcl::PointCloud<pcl::PointXYZ> > clustersPointCloudVec ;
        heuristic->clusteringPointCloud(clustersPointCloudVec,diffPtr);

        ///////////Add Accuracy clusters///////////////
        for(int i =0; i<accuracyClusters.size(); i++)
        {
            clustersPointCloudVec.push_back(accuracyClusters[i]);
        }
        std::cout<<"\n\n////////////// number of clusters: "<<clustersPointCloudVec.size()<<" ////////////////////\n"<<std::endl;
        accuracyClusters.erase(accuracyClusters.begin(),accuracyClusters.end());

        /////////////loop through the clusters and perform discretization and filtering//////////
        res -= resDecrement;
        connRadius = std::sqrt((res*res) + (res*res)) + 0.01;
        //std::cout<<"resolution updated: "<<res;
        if(res>0)
        {
            pcl::PointCloud<pcl::PointXYZ> clustersCloudHull;
            for (int i =0 ; i<clustersPointCloudVec.size(); i++)
            {
                pcl::PointCloud<pcl::PointXYZ> cluster;
                cluster.points = clustersPointCloudVec[i].points;
                geometry_msgs::Pose clusterGridStart;
                geometry_msgs::Vector3 clusterGridSize;
                heuristic->findClusterBB(cluster,clusterGridSize,clusterGridStart);

                //discretize & filtering
                this->generateRegularGrid(clusterGridStart, clusterGridSize,res,true,45,true,true);


                //connect internally
                if(robotFilteredPoses.poses.size()>1) //check does this problem appear and remove this unnecessary if
                {
                    SearchSpaceNode * S =insertTempSearchSpace(robotFilteredPoses,sensorsFilteredPoses);
                    this->connectClustersInternalNodes(S,connRadius);
                    freeTempSearchSpace(S);
                }

                //find the outerpoints of the cluster
                //find the outerpoints of the cluster (convex/concave hull is not so accurate returns warning and the points returned are not exactly the points from the clusters, there is a precesion problem)
                //heuristic->findClusterOuterPoints(robotFilteredPoses,clustersCloudHull);

                //another suggestion instead of the outer points: connect the clusters points with the nearest point of the previous stage cluster
                pcl::PointCloud<pcl::PointXYZ> positionsCloud;
                for(int i =0; i<robotFilteredPoses.poses.size(); i++)
                {
                    pcl::PointXYZ pt;
                    pt.x = robotFilteredPoses.poses[i].position.x;
                    pt.y = robotFilteredPoses.poses[i].position.y;
                    pt.z = robotFilteredPoses.poses[i].position.z;
                    positionsCloud.push_back(pt);
                }
                clustersCloudHull += positionsCloud;

                robotFilteredPoses.poses.erase(robotFilteredPoses.poses.begin(), robotFilteredPoses.poses.end());
                sensorsFilteredPoses.erase(sensorsFilteredPoses.begin(), sensorsFilteredPoses.end());

            }

            //connect the clusters hull with the previous step cluster (nearest neighbor)
            this->connectToNN(hullCloud1,clustersCloudHull);
            hullCloud1.points = clustersCloudHull.points;


        } else break;//end of if statement

        clustersPointCloudVec.erase(clustersPointCloudVec.begin(), clustersPointCloudVec.end());

    }//end of dynamic sampling loop
    std::cout<<"\n	--->>> NODES CONNECTED <<<---	Total number of connections:"<<this->MAXNODES<<std::endl;
}

std::vector<geometry_msgs::Point>  PathPlanner::getConnections()
{
    std::vector<geometry_msgs::Point> lineSegments;
    geometry_msgs::Point pt;
    int num=0;
    SearchSpaceNode *temp = searchspace;
    while (temp != NULL)
    {
        for(int i=0; i < temp->children.size();i++)
        {
            num++;
            pt.x = temp->location.position.x;
            pt.y = temp->location.position.y;
            pt.z = temp->location.position.z;
            lineSegments.push_back(pt);

            pt.x = temp->children[i]->location.position.x;
            pt.y = temp->children[i]->location.position.y;
            pt.z = temp->children[i]->location.position.z;
            lineSegments.push_back(pt);
        }
        temp = temp->next;
    }
    return lineSegments;
}

}
