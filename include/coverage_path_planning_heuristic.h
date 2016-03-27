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
#ifndef CPP_HEURISTICT_H_
#define CPP_HEURISTICT_H_

#include <QString>
#include <QHash>
#include "ssppexception.h"
#include "node.h"
#include <component_test/occlusion_culling_gpu.h>
#include <component_test/occlusion_culling.h>
#include "heuristic_interface.h"

using namespace std;

namespace SSPP
{

class CoveragePathPlanningHeuristic:public Heuristic
{
public:
    CoveragePathPlanningHeuristic(ros::NodeHandle &n, string modelName, bool d=false);
    ~CoveragePathPlanningHeuristic();
    bool isCost();
    void setCoverageTarget(double coverageTarget);
    void setCoverageTolerance(double coverageTolerance);
    void setDebug(bool debug);
    void calculateHeuristic(Node *n);
    bool terminateConditionReached(Node *node);

private:
    OcclusionCullingGPU* obj;
    bool debug;
    double coverageTarget;
    double coverageTolerance;
};

}
#endif /*CPP_HEURISTICT_H_*/
