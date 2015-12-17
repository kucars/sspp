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
#ifndef HEURISTICT_H_
#define HEURISTICT_H_

#include <QString>
#include <QHash>
#include "ssppexception.h"
#include "node.h"
#include <component_test/occlusion_culling.h>
using namespace std;

namespace SSPP
{

class Heuristic
{
public:
	virtual ~Heuristic(){}
	virtual double gCost(Node *)=0;
    virtual double hCost(Node *,Node * )=0;
    virtual double hCost(Node *)=0;
//    double hCovCost(Node *n);
    static Heuristic * factory(QString type) throw(SSPPException);
    static Heuristic * factory(QString type,QHash<QString, int> *) throw(SSPPException);
};

class SocialHeuristic : public Heuristic
{
public:
	QHash<QString, int> *socialRewards;
	SocialHeuristic(QHash<QString, int> *socRew){this->socialRewards=socRew;}
	friend class Heuristic;
public:
	double gCost(Node *n);
    double hCost(Node *n, Node * end);
    double hCost(Node *n){}
	~SocialHeuristic(){}
};

class DistanceHeuristic : public Heuristic
{
public:
	DistanceHeuristic(){}
	friend class Heuristic;
public:
	double gCost(Node *n);
    double hCost(Node *n, Node * end);
    double hCost(Node *n){}
	~DistanceHeuristic(){}
};

class SurfaceCoverageHeuristic : public Heuristic
{
public:
    SurfaceCoverageHeuristic(){ obj = new OcclusionCulling("scaled_desktop.pcd");}
    friend class Heuristic;
public:
    double gCost(Node *n);
    double hCost(Node *n, Node * end){}
    double hCost(Node *n);
    OcclusionCulling* obj;
    ~SurfaceCoverageHeuristic(){}
};

}
#endif /*HEURISTICT_H_*/
