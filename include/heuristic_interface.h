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
#include <component_test/occlusion_culling_gpu.h>
#include <component_test/occlusion_culling.h>

using namespace std;

namespace SSPP
{

class Heuristic
{
public:
    Heuristic(){}
    virtual ~Heuristic(){}
    virtual void calculateHeuristic(Node *)=0;
    virtual bool terminateConditionReached(Node *)=0;
    /*!
     * \brief isCost
     * \return
     * Defines whether this heuristic function is a cost or reward function
     */
    virtual bool isCost()=0;
};

}
#endif /*HEURISTICT_H_*/
