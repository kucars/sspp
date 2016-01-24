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
#include "heuristic.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
namespace SSPP
{

Heuristic *  Heuristic::factory(QString type) throw(SSPPException)
{
    if (type == "Distance")
    {
        std::cout<<"\nUsing Distance Heuristic";
        return new DistanceHeuristic;
    }
    else if (type == "SurfaceCoverage")
    {
        std::cout<<"\nUsing Surface Coverage Heuristic";
        return new SurfaceCoverageHeuristic;
    }
    else if (type == "SurfaceCoveragewithOrientation")
    {
        std::cout<<"\nUsing Surface Coverage with orientaion Heuristic";
        return new SCwithOrientationHeuristic;
    }
    throw (SSPPException((char*)"Bad Heuristic Type"));
}

Heuristic *  Heuristic::factory(QString type,QHash<QString, int> *soRe) throw(SSPPException)
{
    if (type == "Social")
    {
        std::cout<<"\nUsing Social Heuristic";
        return new SocialHeuristic(soRe);
    }
    throw (SSPPException((char*)"Bad Heuristic Type"));
}

double SocialHeuristic::gCost(Node *n)
{
    return 0;
}

double SocialHeuristic::hCost(Node *n,Node * end)
{
    double cost=0,h=0,g=0;
    if(n == NULL || n->parent==NULL)
        return 0.0;
    // Using the Euclidean distance
    h = Dist(end->pose.p,n->pose.p);
    g = n->parent->g_value + Dist(n->pose.p,n->parent->pose.p);
    cost =  h + g;
    cost -= this->socialRewards->value(QString("%1-%2-%3").arg(n->parent->id).arg(n->id).arg(end->id));
    return cost;
}

double DistanceHeuristic::gCost(Node *n)
{
    double cost;
    if(n == NULL || n->parent==NULL)
        return 0.0;
    cost = n->parent->g_value + Dist(n->pose.p,n->parent->pose.p);
    return cost;
}

double DistanceHeuristic::hCost(Node *n,Node * end)
{
    double h=0,angle_cost=0,obstacle_penalty=0,reverse_penalty=0,delta_d=0;
    if(n == NULL)
        return(0);
    // Using the Euclidean distance
    h = Dist(end->pose.p,n->pose.p);
    //h = 0;
    if (n->parent != NULL) // Adding the Angle cost, we have to uniform the angle representation to the +ve rep or we well get a non sense result
    {
        double a,b;
        a = n->pose.phi;
        b = n->parent->pose.phi;
        angle_cost = fabs(anglediffs(a,b)); // in radians
        delta_d = Dist(n->pose.p,n->parent->pose.p);
    }
    obstacle_penalty = n->nearest_obstacle;
    if(n->direction == BACKWARD)
        reverse_penalty = delta_d;

    // 0.555 is the AXLE Length
    return ( h*(1 + reverse_penalty ) + 0.555 * angle_cost + obstacle_penalty*delta_d);
}

double SurfaceCoverageHeuristic::gCost(Node *n)
{
    double cost;
    if(n == NULL || n->parent==NULL)
        return 0.0;
//    cost = n->parent->g_value + Dist(n->pose.p,n->parent->pose.p);//
    cost = 0.0;// it should be accumulating coverage
    return cost;
}

double SurfaceCoverageHeuristic::hCost(Node *n)
{
    double h=0,d,c;
    if(n == NULL)
        return(0);
    // Using the coverage percentage

    ros::Time heuristic_begin = ros::Time::now();

    std::cout<<"\nchild collective cloud after filtering size: "<<n->cloud_filtered->size()<<"\n";

    std::cout<<"parent distance :"<<n->parent->distance<<" current node distance: "<<n->distance<<"\n";
    std::cout<<"parent coverage :"<<n->parent->coverage<<" current node coverage: "<<n->coverage<<"\n";

    d= Dist(n->pose.p,n->parent->pose.p);
    std::cout<<"Calculated local distance d:"<<d<<" comulative distance: "<<n->distance<<"\n";

    c= n->coverage - n->parent->coverage;
    std::cout<<"extra coverage c : "<<c<<"\n";
//    h=n->coverage;//only coverage
    if(d!=0.0)
        h = n->parent->h_value + ((1/d)*c); //distance and coverage heuristic ;
    else h = n->parent->h_value + c;
    std::cout<<"parent h value :"<<n->parent->h_value<<" h value calculation: "<<h<<"\n";
    ros::Time heuristic_end = ros::Time::now();
    double elapsed =  heuristic_end.toSec() - heuristic_begin.toSec();
    std::cout<<"HEURISTIC duration (s) = "<<elapsed<<"\n";
    return h;
}

double SCwithOrientationHeuristic::gCost(Node *n)
{
    double cost;
    if(n == NULL || n->parent==NULL)
        return 0.0;
//    cost = n->parent->g_value + Dist(n->pose.p,n->parent->pose.p);//
    cost = 0.0;// it should be accumulating coverage
    return cost;
}

double SCwithOrientationHeuristic::hCost(Node *n)
{
    double h=0,d,c;
    if(n == NULL)
        return(0);
    // Using the coverage percentage

    ros::Time heuristic_begin = ros::Time::now();

    std::cout<<"\nchild collective cloud after filtering size: "<<n->cloud_filtered->size()<<"\n";
    std::cout<<"parent distance :"<<n->parent->distance<<" current node distance: "<<n->distance<<"\n";
    std::cout<<"parent coverage :"<<n->parent->coverage<<" current node coverage: "<<n->coverage<<"\n";

    d= Dist(n->pose.p,n->parent->pose.p);
    std::cout<<"Calculated local distance d:"<<d<<" comulative distance: "<<n->distance<<"\n";

    c= n->coverage - n->parent->coverage; //extra coverage
    std::cout<<"extra coverage c : "<<c<<"\n";
//    h=n->coverage;//only coverage
    if(d!=0.0)
    {
//        std::cout<<"distance and coverage heuristic\n";
        h = n->parent->h_value + ((1/d)*c); //distance and coverage heuristic ;
    }
    else
    {
//        std::cout<<"###################################coverage heuristic##################################################\n";
        tf::Quaternion qt(n->parent->pose.p.orientation.x,n->parent->pose.p.orientation.y,n->parent->pose.p.orientation.z,n->parent->pose.p.orientation.w);
        tf::Quaternion qt_n(n->pose.p.orientation.x,n->pose.p.orientation.y,n->pose.p.orientation.z,n->pose.p.orientation.w);
        //using utils angle diff
//        double a,b,angle_diff;
//        a=tf::getYaw(qt);
//        b=tf::getYaw(qt_n);
//        angle_diff = anglediffs(a,b);
//        std::cout<<"###angle way (1.1) between the parent and child###: "<<angle_diff<<"\n";
//        angle_diff = anglediff(a,b);
//        std::cout<<"###angle way (1.2) between the parent and child###: "<<angle_diff<<"\n";

        // using quetranion
        double angle, normAngle;
        angle=qt.angleShortestPath(qt_n);
        normAngle=1-angle/(2*M_PI);
        std::cout<<"###angle way (2) between the parent and child###: "<<angle<<"\n";
        h = n->parent->h_value + normAngle*c;
    }

    std::cout<<"parent h value :"<<n->parent->h_value<<" h value calculation: "<<h<<"\n";
    ros::Time heuristic_end = ros::Time::now();
    double elapsed =  heuristic_end.toSec() - heuristic_begin.toSec();
    std::cout<<"HEURISTIC duration (s) = "<<elapsed<<"\n";
    return h;
}


}
