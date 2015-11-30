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

PathPlanner::PathPlanner(ros::NodeHandle &n, Robot *rob, double dG, double bridge_len,
                         double bridge_r, double reg_g, double conn_rad, double obst_pen, double bridge_conn_rad_in):
    nh(n),
    Astar(n,rob,dG,"Distance"),
    map_initialized(false),
    obstacle_expansion_radius(rob->expansionRadius),
    bridge_length(bridge_len),
    bridge_res(bridge_r),
    regGridRes(reg_g),
    reg_grid_conn_rad(conn_rad),
    obst_penalry_radius(obst_pen),
    bridge_conn_rad(bridge_conn_rad_in)
{
    treePub = n.advertise<visualization_msgs::Marker>("search_tree", 10);
    planningSteps = 0;
    obstaclesExpanded  = false;
    originalMap        = NULL;
}

PathPlanner :: ~PathPlanner()
{
    freeResources();
    std::cout<<"\n	--->>> Allocated Memory FREED <<<---";
}

void PathPlanner::freeResources()
{
    freeSearchSpace();
    freePath();
    p=root=test=NULL;
    /*
    if(originalMap)
        delete originalMap;
    if(Astar::map)
        delete Astar::map;
    */
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

unsigned int PathPlanner::getPlanningSteps()
{
    return planningSteps;
}

void   PathPlanner::setExpRad(double a)
{
    obstacle_expansion_radius = a;
}

void   PathPlanner::setBridgeLen(double a)
{
    bridge_length = a;
}

void   PathPlanner::setBridgeRes(double a)
{
    bridge_res = a;
}

void   PathPlanner::setRegGrid(double a)
{
    regGridRes = a;
}

void   PathPlanner::setConRad(double a)
{
    reg_grid_conn_rad = a;
}

void   PathPlanner::setObstDist(double a)
{
    obst_penalry_radius = a;
}

// Nifty way of doing an obstacle expansion, not finalized, ----->>>>> @ TO DO @ <<<<<------
void PathPlanner::expandObstacles()
{
    if(!map_initialized)
    {
        std::cout<<"\n   --->>> You have to read the map before Expanding the Obstacles <<<---";
        return;
    }
    if(obstaclesExpanded)
    {
        std::cout<<"\n    --->>> Obstacles have been expanded before, Skipping <<<---";
        delete Astar::map;
        Astar::map = originalMap->clone();
    }
    int thickness;
    int m,n,x,y,radius;
    thickness = int(this->obstacle_expansion_radius/map->mapRes);
    radius =2; // This covers vertical, horizontal and diagonal cells
    Map temp_map(map->width,map->height,map->mapRes,QPointF(map->width/2.0,map->height/2.0),Pose(0,0,0,0));
    for(int i = 0;i<map->width;i++)
    {
        for(int j = 0;j<map->height;j++)
        {
            temp_map.grid[i][j] = map->grid[i][j];
        }
    }
    for(int i=0;i<this->map->width - 1;i++)
    {
        for(int j=0;j<this->map->height - 1 ;j++)
        {
            for(x = (i - radius) ; x <= (i + radius) ; x ++)
            {
                if (x < 0) x = 0;
                if (x >= this->map->width)  break;
                y = (int)(- sqrt(radius*radius - (x - i)*(x - i)) + j);
                if (y < 0) y = 0;
                if (y >= this->map->height) y = this->map->height - 1;
                if (temp_map.grid[i][j] && !temp_map.grid[x][y])
                {
                    for (int r = 1; r <(int)(thickness);r++)
                        for( m = (int)(i - r); m <= (int)(i + r);m++)
                        {
                            if (m < 0) m = 0;
                            if (m >= this->map->width)  break;
                            n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
                            if (n < 0) n = 0;
                            if (n >= this->map->height) n = this->map->height - 1;
                            this->map->grid[m][n] = true;
                            n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
                            if (n < 0) n = 0;
                            if (n >= this->map->height) n = this->map->height - 1;
                            this->map->grid[m][n] = true;
                        }
                    break;
                }
                y = (int)(+ sqrt(radius*radius - (x - i)*(x - i)) + j);
                if (y < 0) y = 0;
                if (y >= this->map->height) y = this->map->height - 1;
                if (temp_map.grid[i][j] && !temp_map.grid[x][y])
                {
                    for (int r = 1; r <(int)(thickness);r++)
                        for( m = (int)(i - r); m <= (int)(i + r);m++)
                        {
                            if (m < 0) m = 0;
                            if (m >= this->map->width)  break;
                            n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
                            if (n < 0) n = 0;
                            if (n >= this->map->height) n = this->map->height - 1;
                            this->map->grid[m][n] = true;
                            n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
                            if (n < 0) n = 0;
                            if (n >= this->map->height) n = this->map->height - 1;
                            this->map->grid[m][n] = true;
                        }
                    break;
                }
            }
        }
    }
    map->savePgm();
    std::cout<<"\n	--->>> OBSTACLES EXPANDED SUCCESSFULLY <<<---	";
    planningSteps|=OBST_EXPAND;
    obstaclesExpanded = true;
}

bool PathPlanner::readSpaceFromFile(const char *filename,unsigned _planningSteps)
{
    double locationx,locationy,locationz,obstacle_cost;
    int type;
    SearchSpaceNode *temp;
    assert(filename != NULL);
    filename = strdup(filename);
    FILE *file = fopen(filename, "r");
    if (!file)
    {
        std::cout<<"\nCan not open Search Space File";
        fclose(file);
        return false;
    }
    // Read metadata line, it's expected to be the first line
    // If planning parameters are not the same as the current then we can't read it
    double obstacle_expansion_radius_t,bridge_length_t,bridge_res_t,regGridRes_t,
            reg_grid_conn_rad_t,obst_penalry_radius_t,bridge_conn_rad_t;
    unsigned int planningSteps_t;
    fscanf(file,"%lf %lf %lf %lf %lf %lf %lf %d\n",&obstacle_expansion_radius_t,&bridge_length_t,&bridge_res_t,&regGridRes_t,
           &reg_grid_conn_rad_t,&obst_penalry_radius_t,&bridge_conn_rad_t,&planningSteps_t);
    if(!isEqual(obstacle_expansion_radius_t,obstacle_expansion_radius) || !isEqual(bridge_length_t,bridge_length) ||
            !isEqual(bridge_res_t,bridge_res) || !isEqual(regGridRes_t,regGridRes) || !isEqual(reg_grid_conn_rad_t,reg_grid_conn_rad) ||
            !isEqual(obst_penalry_radius_t,obst_penalry_radius) || !isEqual(bridge_conn_rad_t,bridge_conn_rad) || planningSteps_t!=_planningSteps)
    {
        std::cout<<"\nMetadata Parameters mismatch, can't read search space, try to generate new one";
        std::cout<<"\nExpected obst exp:"<<obstacle_expansion_radius<<" _t:"<<obstacle_expansion_radius_t;
        std::cout<<"\nExpected bridge_l:"<<bridge_length<<" found:"<<bridge_length_t;
        std::cout<<"\nExpected bridge_res:"<<bridge_res<<" found:"<<bridge_res_t;
        std::cout<<"\nExpected reg_grid_res:"<<regGridRes<<" found:"<<regGridRes_t;
        std::cout<<"\nExpected reg_grid_con:"<<reg_grid_conn_rad<<" found:"<<reg_grid_conn_rad_t;
        std::cout<<"\nExpected obst_pen:"<<obst_penalry_radius<<" found:"<<obst_penalry_radius_t;
        std::cout<<"\nExpected bridge_con:"<<bridge_conn_rad<<" found:"<<bridge_conn_rad_t;
        std::cout<<"\nExpected planningSteps:"<<_planningSteps<<" found:"<<planningSteps_t;
        fclose(file);
        return false;
    }
    // overwrite the planning steps to reflect those in the file
    planningSteps = _planningSteps;
    while (!feof(file))
    {
        fscanf(file,"%lf %lf %lf %lf %d\n",&locationx,&locationy,&locationz,&obstacle_cost,&type);
        if (search_space == NULL ) // Constructing the ROOT NODE
        {
            temp = new SearchSpaceNode;
            temp->location.position.x = locationx;
            temp->location.position.y = locationy;
            temp->location.position.z = locationz;
            temp->obstacle_cost = obstacle_cost;
            temp->parent   = NULL;
            temp->next     = NULL;
            temp->type     = type;
            search_space = temp;
        }
        else
        {
            temp = new SearchSpaceNode;
            temp->location.position.x = locationx;
            temp->location.position.y = locationy;
            temp->location.position.z = locationz;
            temp->obstacle_cost = obstacle_cost;
            temp->parent = NULL;
            temp->next   = search_space;
            temp->type     = type;
            search_space = temp;
        }
    }
    fclose(file);
    return true;
}

bool PathPlanner::saveSpace2File(const char *filename)
{
    assert(filename != NULL);
    filename = strdup(filename);
    FILE *file = fopen(filename, "wb");
    if (!file)
    {
        std::cout<<"\nCan not open Search Space File";
        fclose(file);
        return false;
    }
    SearchSpaceNode *temp=search_space;
    //obstacle_expansion_radius,bridge_length,bridge_res,regGridRes,reg_grid_conn_rad,obst_penalry_radius,bridge_conn_rad;
    // Save metadata
    fprintf(file,"%lf %lf %lf %lf %lf %lf %lf %d\n",obstacle_expansion_radius,bridge_length,bridge_res,regGridRes,
            reg_grid_conn_rad,obst_penalry_radius,bridge_conn_rad,planningSteps);
    while (temp)
    {
        fprintf(file,"%f %f %f %f %d\n",temp->location.position.x,temp->location.position.x,temp->location.position.z,temp->obstacle_cost,temp->type);
        temp = temp->next;
    }
    fclose(file);
    return true;
}

void PathPlanner::generateRegularGrid()
{
    SearchSpaceNode *temp;
    if(!map_initialized)
    {
        std::cout<<"\n	--->>> You have to read the map First <<<---	";
        return;
    }
    geometry_msgs::Pose p;
    for(int i=0; i<this->map->width; i++)
    {
        for(int j=0;j<this->map->height;j++)
        {
            if (!this->map->grid[i][j]) //Free Pixel
            {
                if (search_space == NULL ) // Constructing the ROOT NODE
                {
                    temp = new SearchSpaceNode;
                    temp->location.position.x = i;
                    temp->location.position.y = j;
                    temp->location.position.z = 0;
                    map->convertPix(&temp->location);
                    temp->parent   = NULL;
                    temp->next     = NULL;
                    temp->type     = RegGridNode;
                    search_space = temp;
                }
                else
                {
                    p.position.x = i;
                    p.position.y = j;
                    map->convertPix(&p);
                    if (checkShortestDistance(p,regGridRes))
                    {
                        temp = new SearchSpaceNode;
                        temp->location.position.x = p.position.x;
                        temp->location.position.y = p.position.y;
                        temp->location.position.z = 0;
                        temp->parent = NULL;
                        temp->next   = search_space;
                        temp->type     = RegGridNode;
                        search_space = temp;
                    }
                }
            }
        }
    }
    std::cout<<"\n	--->>> REGULAR GRID GENERATED SUCCESSFULLY <<<---	";
    planningSteps|=REGULAR_GRID;
}

void PathPlanner::bridgeTest()
{
    int radius,pixels_per_bridge;
    SearchSpaceNode *temp;
    geometry_msgs::Pose p;
    double x,y,x2,y2;
    pixels_per_bridge = (int) (robot->robotLength/map->mapRes);
    radius = (int) (robot->robotLength/(map->mapRes*2.0));
    for(int i=0; i < this->map->width - pixels_per_bridge; i++)
    {
        for(int j=0;j<this->map->height - pixels_per_bridge ;j++)
        {
            int ab=0;
            for( x = (i - radius) ; x < (i + radius) ; x+=(radius/4.0))//x+=(radius))
            {
                // The circle is of Center (i,j) and radius R=radius
                ab++;
                if (x < 0) x = 0;
                if (x > this->map->width) break;
                y = (int)(+ sqrt(radius*radius -  ( x - i)*( x - i)) + j);
                if (y < 0) y = 0;
                if (y >= this->map->height) y = this->map->height - 1;

                x2 = i + (i - x);
                if (x2 < 0) x2 = 0;
                if (x2 > this->map->width) break;
                y2 = (int)(- sqrt(radius*radius - (x2 - i)*(x2 - i)) + j);
                if (y2 < 0) y2 = 0;
                if (y2 >= this->map->height) y2 = this->map->height - 1;
                if (!this->map->grid[i][j]&&this->map->grid[(int)x][(int)y]&&this->map->grid[(int)x2][(int)y2])
                {
                    p.position.x = i;
                    p.position.y = j;
                    p.position.z = 0;
                    map->convertPix(&p);
                    if (search_space == NULL ) // Constructing the ROOT NODE
                    {
                        temp = new SearchSpaceNode;
                        temp->location.position.x = p.position.x;
                        temp->location.position.y = p.position.y;
                        temp->location.position.z = p.position.z;
                        temp->parent   = NULL;
                        temp->next     = NULL;
                        temp->type     = BridgeNode;
                        search_space = temp;
                    }
                    else
                    {
                        if (checkShortestDistance(p,bridge_res))
                        {
                            temp = new SearchSpaceNode;
                            temp->location.position.x = p.position.x;
                            temp->location.position.y = p.position.y;
                            temp->location.position.z = p.position.z;
                            temp->parent = NULL;
                            temp->next   = search_space;
                            temp->type     = BridgeNode;
                            search_space = temp;
                        }
                    }
                }
            }
        }
    }
    std::cout<<"\n	--->>> BRIDGE TEST FINISHED SUCCESSFULLY <<<---";
    planningSteps|=BRIDGE_TEST;
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

        cout <<"Step [" << step++ <<"] x["<< pixel.position.x<<"] y["<<pixel.position.y<<"] z["<<pixel.position.z<< "] Direction="<<p->direction;
        std::cout<<"\n\tG cost="<<p->g_value<<"\tH cost="<<p->h_value<<"\tFcost="<<p->f_value;
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

void PathPlanner::addCostToNodes()
{
    SearchSpaceNode * S;
    geometry_msgs::Pose  point;
    double number_of_pixels,radius,nearest_obstacle;
    int i,j;
    //int n=0;
    if (!search_space) // Do nothing if Search Space is not Yet Created
        return;
    S = search_space;
    number_of_pixels = obst_penalry_radius / map->mapRes;
    while (S!=NULL)
    {
        point.position.x = S->location.position.x;
        point.position.y = S->location.position.y;
        point.position.z = S->location.position.z;
        map->convert2Pix(&point);
        nearest_obstacle = 0;

        for(radius = (int)number_of_pixels ; radius >0 ; radius --)
        {
            for( i= (int)(point.position.x - radius) ; i < (int)(point.position.x + radius); i+=1)
            {
                if (i < 0) i = 0;
                if (i >= this->map->width)  break;
                j = (int)(- sqrt(radius*radius - (i - point.position.x)*(i - point.position.x)) + point.position.y);
                if (j < 0) j = 0;
                if (j >= this->map->height) j = this->map->height - 1;
                if(this->map->grid[i][j])
                    nearest_obstacle = radius;
                j = (int)(+ sqrt(radius*radius - (i - point.position.x)*(i - point.position.x)) + point.position.y);
                if (j < 0) j = 0;
                if (j >= this->map->height) j = this->map->height - 1;

                if(this->map->grid[i][j])
                    nearest_obstacle = radius;
            }
        }
        // this is a normalized cost, it will make more sense later on
        S->obstacle_cost =  (obst_penalry_radius - nearest_obstacle*map->mapRes)/obst_penalry_radius;
        //S->obstacle_cost = 0;
        S = S->next;
    }
    std::cout<<"\n	--->>> Penalty Added <<<---	";
    planningSteps|=OBST_PENALTY;
}

void PathPlanner::connectNodes()
{
    SearchSpaceNode * S;
    SearchSpaceNode *temp;
    double distance,angle;
    if (!search_space) // Do nothing if Search Space is not Yet Created
        return;
    temp = search_space;
    //	cout<<"\n RegGrid Conn:"<<reg_grid_conn_rad<<" Bridge Conn:"<<bridge_conn_rad;
    while (temp!=NULL)
    {
        S = search_space;
        while (S!=NULL)
        {
            distance = Dist(S->location,temp->location);
            double testDist;
            if(!(S->type==RegGridNode && temp->type==RegGridNode))
            {
                testDist = bridge_conn_rad;
            }
            else
            {
                testDist = reg_grid_conn_rad;
            }

            if (distance <= testDist && distance !=0)
            {
                angle = atan2(S->location.position.y - temp->location.position.y ,S->location.position.x - temp->location.position.x);
                if(!inObstacle(temp->location,angle) && !inObstacle(S->location,angle))
                {
                    temp->children.push_back(S);
                }
            }
            S = S->next;
        }
        temp = temp->next;
    }
    std::cout<<"\n	--->>> NODES CONNECTED <<<---	";
    planningSteps|=NODES_CONNECT;
}

bool PathPlanner::checkShortestDistance(geometry_msgs::Pose p,double neigbhour_distance)
{
    SearchSpaceNode * S;
    double distance,shortest_distance = 10000000;

    S = search_space;
    while (S!=NULL)
    {
        distance = sqrt(pow(S->location.position.x - p.position.x,2) + pow(S->location.position.y - p.position.y,2) + pow(S->location.position.z - p.position.z,2));
        if (distance <= shortest_distance)
            shortest_distance = distance;
        S = S->next;
    }
    if( shortest_distance > neigbhour_distance )
        return 1;
    else
        return 0;
}

void PathPlanner::showConnections()
{
    geometry_msgs::Pose loc1,loc2;
    SearchSpaceNode *temp = search_space;
    int m=0,n=0;
    while (temp != NULL)
    {
        for(int i=0; i < temp->children.size();i++)
        {
            loc1 = temp->location;
            map->convert2Pix(&loc1);
            loc2 = temp->children[i]->location;
            map->convert2Pix(&loc2);
            m++;
        }
        temp = temp->next;
        n++;
    }
    std::cout<<"\n"<<QString("\n---->>> TOTAL NUMBER OF CONNECTIONS =%1\n---->>> Total Nodes in search Space =%2").arg(m).arg(n).toStdString();
    this->MAXNODES = 2*m;
}

void PathPlanner::saveSearchSpace()
{
    geometry_msgs::Pose p;
    SearchSpaceNode *temp = search_space;
    while (temp != NULL)
    {
        p = temp->location;
        map->convert2Pix(&p);
        this->map->grid[int(p.position.x)][(int)p.position.y]= true ;
        temp =temp->next;
    }
}

void PathPlanner::updateMap(Map *mapPatch)
{
    geometry_msgs::Pose p;
    for(int i=0;i<mapPatch->pointCloud.size();i++)
    {
        QPointF mp;
        mp = mapPatch->pointCloud[i];
        p.position.x = mp.x();
        p.position.y = mp.y();
        p.position.z = 0;
        map->convert2Pix(&p);
        if(this->map->grid[int(p.position.x)][(int)p.position.y] == false)
            this->map->grid[int(p.position.x)][(int)p.position.y]= false;
    }
    SearchSpaceNode *temp = search_space;
    while (temp != NULL)
    {
        p = temp->location;
        map->convert2Pix(&p);
        this->map->grid[int(p.position.x)][(int)p.position.y]= true ;
        temp =temp->next;
    }
}

/*! Sets the converted QImage or the Laser Scan Map to
 * the Current Planner
 */
void PathPlanner::setMap(Map * map_in)
{
    if(!map_in)
    {
        std::cout<<"\nWhy the hell ur giving me an empty map ???";
        fflush(stdout);
        return;
    }
    if(!map_in->grid)
    {
        std::cout<<"\nWhy the hell ur giving me an empty map data ???";
        fflush(stdout);
        return;
    }
    Astar::map        = map_in->clone();
    this->originalMap = map_in->clone();
    MAXNODES = MaxLong;//map->height*map->width*map->width*map->height;
    map_initialized = true;
}

}
