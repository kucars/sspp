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
#include "pathplanner.h"
namespace SSPP
{

PathPlanner::PathPlanner(ros::NodeHandle &n, Robot *rob, double conn_rad):
    nh(n),
    Astar(n,rob),
    reg_grid_conn_rad(conn_rad),
    sampleOrientations(false)
{
    treePub             = nh.advertise<visualization_msgs::Marker>("search_tree", 10);
    connectionPub       = nh.advertise<visualization_msgs::Marker>("connectivity", 10);
    searchSpacePub      = nh.advertise<visualization_msgs::Marker>("search_space", 100);
    pathPub             = nh.advertise<visualization_msgs::Marker>("path_testing", 100);
    pathPointPub        = nh.advertise<visualization_msgs::Marker>("path_point", 100);
    testPointPub        = nh.advertise<visualization_msgs::Marker>("test_point", 100);
    coveredPointsPub    = nh.advertise<sensor_msgs::PointCloud2>("gradual_coverage", 100);
    connectionDebugPub  = nh.advertise<visualization_msgs::Marker>("debug", 10);

    pathDir = ros::package::getPath("component_test");
    std::string str = pathDir+"/src/mesh/etihad_nowheels.obj";
    loadOBJFile(str.c_str(), p1, triangles);
    tree_cgal = new Tree1(triangles.begin(),triangles.end());

}

PathPlanner :: ~PathPlanner()
{
    freeResources();
    std::cout<<"\n	--->>> Allocated Memory FREED <<<---"<<std::endl;
}

void PathPlanner::freeResources()
{
    freeSearchSpace();
    freePath();
    p=root=test=NULL;

}

void PathPlanner::freePath()
{
    while(path != NULL)
    {
        p = path->next;
        delete path;
        path = p;
    }
}

void   PathPlanner::setConRad(double a)
{
    reg_grid_conn_rad = a;
}

void PathPlanner::generateRegularGrid(geometry_msgs::Pose gridStartPose,geometry_msgs::Vector3 gridSize, float gridRes, bool sampleOrientations)
{
    gridResolution = gridRes;
    this->sampleOrientations = sampleOrientations;
    generateRegularGrid(gridStartPose,gridSize);
}

void PathPlanner::generateRegularGrid(geometry_msgs::Pose gridStartPose,geometry_msgs::Vector3 gridSize, float gridRes)
{
    gridResolution = gridRes;
    generateRegularGrid(gridStartPose,gridSize);
}

void PathPlanner::generateRegularGrid(geometry_msgs::Pose gridStartPose, geometry_msgs::Vector3 gridSize)
{
    geometry_msgs::Pose pose;
    geometry_msgs::Pose correspondingSensorPose;
    int numSamples = 0;
    for(float x = gridStartPose.position.x; x<=(gridStartPose.position.x + gridSize.x); x+=gridResolution)
        for(float y = gridStartPose.position.y; y<=(gridStartPose.position.y + gridSize.y); y+=gridResolution)
            for(float z = gridStartPose.position.z; z<=(gridStartPose.position.z + gridSize.z); z+=gridResolution)
            {
                pose.position.z=z;
                pose.position.y=y;
                pose.position.x=x;
                //TODO:: add corresponding sensor transformation for CPP
                if(sampleOrientations)
                {
                    int orientationsNum= 360.0f/orientationResolution;
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
                        insertNode(pose,correspondingSensorPose);
                        numSamples++;
                    }
                }
                else
                {
                    pose.orientation.x=0;pose.orientation.y=0;pose.orientation.z=0;pose.orientation.w=1;
                    insertNode(pose,correspondingSensorPose);
                    numSamples++;
                }

            }
    std::cout<<"\n	--->>> REGULAR GRID GENERATED SUCCESSFULLY <<<--- Samples:"<<numSamples++;;
}

//***************linked list based search space*************
void PathPlanner::loadRegularGrid(const char *filename1, const char *filename2)
{
    geometry_msgs::Pose pose;
    geometry_msgs::Pose correspondingSensorPose;
    double locationx,locationy,locationz,qx,qy,qz,qw;
    double senLocx,senLocy,senLocz,senqx,senqy,senqz,senqw;

    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");

    assert(filename2 != NULL);
    filename2 = strdup(filename2);
    FILE *file2 = fopen(filename2, "r");
    if (!file1 || !file2)
    {
        std::cout<<"\nCan not open Search Space File";
        fclose(file1);
        fclose(file2);
    }
    while (!feof(file1) && !feof(file2))
    {
        fscanf(file1,"%lf %lf %lf %lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qx,&qy,&qz,&qw);
        fscanf(file2,"%lf %lf %lf %lf %lf %lf %lf\n",&senLocx,&senLocy,&senLocz,&senqx,&senqy,&senqz,&senqw);

        pose.position.x = locationx;
        pose.position.y = locationy;
        pose.position.z = locationz;
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;

        correspondingSensorPose.position.x = senLocx;
        correspondingSensorPose.position.y = senLocy;
        correspondingSensorPose.position.z = senLocz;
        correspondingSensorPose.orientation.x = senqx;
        correspondingSensorPose.orientation.y = senqy;
        correspondingSensorPose.orientation.z = senqz;
        correspondingSensorPose.orientation.w = senqw;

        insertNode(pose,correspondingSensorPose);
    }
    fclose(file1);
    fclose(file2);
    std::cout<<"\n	--->>> REGULAR GRID GENERATED SUCCESSFULLY <<<---	";
}

void  PathPlanner::showSearchSpace()
{
    SearchSpaceNode *temp = searchspace;
    int n=0;
    while (temp != NULL)
    {
        geometry_msgs::Point pt;
        pt.x= temp->location.position.x;
        pt.y= temp->location.position.y;
        pt.z= temp->location.position.z;
        pts.push_back(pt);
        temp = temp->next;
        n++;
    }
    visualization_msgs::Marker points_vector = drawpoints(pts);
    searchSpacePub.publish(points_vector);
    std::cout<<"\n"<<QString("\n---->>> Total Nodes in search Space =%1").arg(n).toStdString();
}

visualization_msgs::Marker PathPlanner::drawpoints(std::vector<geometry_msgs::Point> points)
{
    visualization_msgs::Marker pointMarkerMsg;
    pointMarkerMsg.header.frame_id="/map";
    pointMarkerMsg.header.stamp=ros::Time::now();
    pointMarkerMsg.ns="point_marker";
    pointMarkerMsg.id = 1000;
    pointMarkerMsg.type = visualization_msgs::Marker::POINTS;
    pointMarkerMsg.scale.x = 0.1;
    pointMarkerMsg.scale.y = 0.1;
    pointMarkerMsg.action  = visualization_msgs::Marker::ADD;
    pointMarkerMsg.lifetime  = ros::Duration(100.0);
    std_msgs::ColorRGBA color;
    color.r = 0.0f; color.g=1.0f; color.b=0.0f, color.a=1.0f;
    std::vector<geometry_msgs::Point>::iterator pointsIterator;
    for(pointsIterator = points.begin();pointsIterator != points.end();pointsIterator++)
    {
        pointMarkerMsg.points.push_back(*pointsIterator);
        pointMarkerMsg.colors.push_back(color);
    }
    return pointMarkerMsg;
}

void PathPlanner :: printNodeList()
{
    int step=1;
    geometry_msgs::Pose  pixel;
    if(!(p = this->path))
        return ;
    std::cout<<"\n--------------------   START OF LIST ----------------------";
    while(p !=NULL)
    {
        pixel.position.x =  p->pose.p.position.x;
        pixel.position.y =  p->pose.p.position.y;
        pixel.position.z =  p->pose.p.position.z;

        std::cout<<"\nStep [" << step++ <<"] x["<< pixel.position.x<<"] y["<<pixel.position.y<<"] z["<<pixel.position.z<< "]";
        std::cout<<"\tG cost="<<p->g_value<<"\tH cost="<<p->h_value<<"\tFcost="<<p->f_value;
        //cout<<"\tStored Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(p->angle);
        if (p->next !=NULL)
        {
            //cout<<"\tNext Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x));
            //cout<<"\tAngle Diff ="<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(anglediff(p->angle,atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x)));
        }
        p = p->next;
    }
    std::cout<<"\n--------------------   END OF LIST ---------------------- ";
}

void PathPlanner::connectNodes()
{
    SearchSpaceNode * S;
    SearchSpaceNode *temp;
    double distance;
    if (!searchspace)
        return;
    temp = searchspace;
    int i = 0;
    while (temp!=NULL)
    {
        S = searchspace;
        while (S!=NULL)
        {
            distance = Dist(S->location,temp->location);
            if (distance <= reg_grid_conn_rad && S != temp)// && distance !=0)
            {
                //collision check
                int intersectionsCount=0;
                //parent
                Point a(temp->location.position.x , temp->location.position.y ,temp->location.position.z );
                //check if parent and child are in the same position (it affects the cgal intersection)
                if (S->location.position.x != temp->location.position.x || S->location.position.y != temp->location.position.y || S->location.position.z != temp->location.position.z )
                {
                    //child
                    Point b(S->location.position.x, S->location.position.y, S->location.position.z);
                    Segment seg_query(a,b);
                    intersectionsCount = tree_cgal->number_of_intersected_primitives(seg_query);
                    if (intersectionsCount==0)
                    {
                        temp->children.push_back(S);
                    }
                    else
                    {
                        std::vector<geometry_msgs::Point> lineSegments3;
                        geometry_msgs::Point pt;
                        pt.x = temp->location.position.x;
                        pt.y = temp->location.position.y;
                        pt.z = temp->location.position.z;
                        lineSegments3.push_back(pt);

                        pt.x = S->location.position.x;
                        pt.y = S->location.position.y;
                        pt.z = S->location.position.z;
                        lineSegments3.push_back(pt);
                        visualization_msgs::Marker linesLists = drawLines(lineSegments3,i,2,100000000,0.08);
                        connectionDebugPub.publish(linesLists);
                        lineSegments3.pop_back();lineSegments3.pop_back();
                        i++;

                    }

                }
                //child and parent are in the same position
                else
                {
                    temp->children.push_back(S);
                }

            }
            S = S->next;
        }
        temp = temp->next;
    }
    std::cout<<"\n	--->>> NODES CONNECTED <<<---	";
    this->MAXNODES = i*searchspace->id;
}

void PathPlanner::showConnections()
{
    std::vector<geometry_msgs::Point> lineSegments1;
    geometry_msgs::Point pt;
    geometry_msgs::Pose loc1,loc2;
    SearchSpaceNode *temp = searchspace;
    int m=0,n=0;
    while (temp != NULL)
    {
        for(int i=0; i < temp->children.size();i++)
        {
            loc1 = temp->location;
            pt.x = temp->location.position.x;
            pt.y = temp->location.position.y;
            pt.z = temp->location.position.z;
            lineSegments1.push_back(pt);

            loc2 = temp->children[i]->location;
            pt.x = temp->children[i]->location.position.x;
            pt.y = temp->children[i]->location.position.y;
            pt.z = temp->children[i]->location.position.z;
            lineSegments1.push_back(pt);
            m++;
        }
        temp = temp->next;
        n++;
    }
    visualization_msgs::Marker linesList = drawLines(lineSegments1,2000000,3,100000000,0.08);
    connectionPub.publish(linesList);
    std::cout<<"\n"<<QString("\n---->>> TOTAL NUMBER OF CONNECTIONS =%1\n---->>> Total Nodes in search Space =%2").arg(m).arg(n).toStdString();
}

void PathPlanner::loadOBJFile(const char* filename, std::vector<Vec3f>& points, std::list<CGALTriangle>& triangles)
{

    FILE* file = fopen(filename, "rb");
    if(!file)
    {
        std::cerr << "file not exist" << std::endl;
        return;
    }

    bool has_normal = false;
    bool has_texture = false;
    char line_buffer[2000];
    while(fgets(line_buffer, 2000, file))
    {
        char* first_token = strtok(line_buffer, "\r\n\t ");
        if(!first_token || first_token[0] == '#' || first_token[0] == 0)
            continue;

        switch(first_token[0])
        {
        case 'v':
        {
            if(first_token[1] == 'n')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_normal = true;
            }
            else if(first_token[1] == 't')
            {
                strtok(NULL, "\t ");
                strtok(NULL, "\t ");
                has_texture = true;
            }
            else
            {
                FCL_REAL x = (FCL_REAL)atof(strtok(NULL, "\t "));
                FCL_REAL y = (FCL_REAL)atof(strtok(NULL, "\t "));
                FCL_REAL z = (FCL_REAL)atof(strtok(NULL, "\t "));
                Vec3f p(x, y, z);
                points.push_back(p);
            }
        }
            break;
        case 'f':
        {
            CGALTriangle tri;
            char* data[30];
            int n = 0;
            while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
            {
                if(strlen(data[n]))
                    n++;
            }

            for(int t = 0; t < (n - 2); ++t)
            {
                if((!has_texture) && (!has_normal))
                {
                    Point p1(points[atoi(data[0]) - 1][0],points[atoi(data[0]) - 1][1],points[atoi(data[0]) - 1][2]);
                    Point p2(points[atoi(data[1]) - 1][0],points[atoi(data[1]) - 1][1],points[atoi(data[1]) - 1][2]);
                    Point p3(points[atoi(data[2]) - 1][0],points[atoi(data[2]) - 1][1],points[atoi(data[2]) - 1][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    //std::cout<<"1: Yep, I get here p1:"<<atoi(data[0]) - 1<<" p2:"<<atoi(data[1]) - 1<<" p2:"<<atoi(data[2]) - 1;
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
                else
                {
                    const char *v1;
                    uint indxs[3];
                    for(int i = 0; i < 3; i++)
                    {
                        // vertex ID
                        if(i == 0)
                            v1 = data[0];
                        else
                            v1 = data[t + i];

                        indxs[i] = atoi(v1) - 1;
                    }
                    Point p1(points[indxs[0]][0],points[indxs[0]][1],points[indxs[0]][2]);
                    Point p2(points[indxs[1]][0],points[indxs[1]][1],points[indxs[1]][2]);
                    Point p3(points[indxs[2]][0],points[indxs[2]][1],points[indxs[2]][2]);
                    tri = CGALTriangle(p1,p2,p3);
                    if(!CGAL::collinear(p1,p2,p3))
                    {
                        triangles.push_back(tri);
                    }
                }
            }
        }
        }
    }
}


}
