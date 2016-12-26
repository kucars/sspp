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
#ifndef PLANNINGMANAGER_H_
#define PLANNINGMANAGER_H_

#include "pathplanner.h"
#include "configfile.h"
#include "mapmanager.h"
#include "robotmanager.h"
#include "utils.h"
#include <QObject>
#include <QString>
#include <QPointF>
#include <sys/types.h>
#include <sys/stat.h>
#include "logger.h"
#include "settings.h"

class RobotManager;
using namespace CasPlanner;
class PlanningManager : public QThread
{
    Q_OBJECT
public:
    enum PLANNINGSTEPS{
        GENERATING_SPACE,
        FINDING_PATH,
        WAITING
    };

    PlanningManager(RobotManager *);
    PlanningManager(RobotManager *,double,double,double,double,double,double,double,double,double);
    ~PlanningManager();
    virtual void run();
    void loadPlanningParameters();
    void setRobotManager(RobotManager *);
    int readConfigs(ConfigFile *cf);
    int setupPlanner();
    int stop();
    PathPlanner * pathPlanner;
    bool renderSearchSpaceTree;
    bool renderSearchTree;
    bool renderPaths;
public Q_SLOTS:
    void  findPath(int coord);
    void  generateSpace();
    void  setStart(Pose);
    void  setEnd(Pose);
    void  setMap(Map *);
    void  setMap(LaserScan laserScan,double local_dist,Pose pose);
    void  setBridgeTest(bool);
    void  setConnNodes(bool);
    void  setRegGrid(bool);
    void  setObstPen(bool);
    void  setExpObst(bool);
    void  setBridgeTestValue(double);
    void  setConnNodesValue(double);
    void  setRegGridValue(double);
    void  setObstPenValue(double);
    void  setExpObstValue(double);
    void  setBridgeResValue(double val);
    void  updateMap(LaserScan laserScan,double local_dist,Pose robotLocation);
    bool  fileExist(const char * fname);
    void  generateSearchSpace(bool loadFromFile,bool overWriteCurrent);
Q_SIGNALS:
    void addMsg(int,int, QString);
    void updateMap(Map* map);
    void searchSpaceGenerated();
    void pathFound(Node*);
protected:
    void findPathState();
    void  generateSearchSpaceState(bool loadFromFile=true,bool overWriteCurrent=false);
    double pixel_res,dist_goal,bridge_len,bridge_res,reg_grid,obst_exp,reg_grid_conn_rad,obst_pen,bridge_conn_rad;
    RobotManager *robotManager;
    bool bridgeTestEnabled,connNodesEnabled,regGridEnabled,obstPenEnabled,expObstEnabled,negate,
        loadSpaceFromFile, overWriteExistingSpace;
    unsigned int planningSteps;
    QPointF rotation_center;
    QString robot_model;
    Pose start,end;
    int executionStep;
    int coord;
};
#endif /*PLANNINGMANAGER_H_*/
