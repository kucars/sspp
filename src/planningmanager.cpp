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
#include "planningmanager.h"
#include "settings.h"
#include "playground.h"
#include "robotmanager.h"
#include "statusbar.h"

/*!
 * Constructor, parameters are provided directly rather than
 * read from the configuration file
 */
PlanningManager::PlanningManager(RobotManager *robMan,
                                 double pixel_res,
                                 double dist_goal,
                                 double bridge_len,
                                 double bridge_res,
                                 double reg_grid,
                                 double obst_exp,
                                 double conn_rad,
                                 double obst_pen,
                                 double bridge_conn_rad
                                      )
{
    this->pixel_res  = pixel_res;
    this->bridge_len = bridge_len;
    this->bridge_res = bridge_res;
    this->reg_grid = reg_grid;
    this->obst_exp = obst_exp;
    this->reg_grid_conn_rad = conn_rad;
    this->obst_pen = obst_pen;
    this->dist_goal = dist_goal;
    this->bridge_conn_rad = bridge_conn_rad;
    this->pathPlanner = NULL;
    this->robotManager = robMan;
    this->connNodesEnabled      = CasPlanner::settings().isConnectNodesEnabled();
    this->regGridEnabled        = CasPlanner::settings().isRegGridEnabled();
    this->obstPenEnabled        = CasPlanner::settings().isObstaclePenEnabled();
    this->expObstEnabled        = CasPlanner::settings().isExpandObstEnabled();
    this->bridgeTestEnabled     = CasPlanner::settings().isBridgeTestEnabled();
    this->renderSearchSpaceTree = false;
    this->renderSearchTree      = false;
    this->renderPaths           = false;
    this->loadSpaceFromFile     = true;
    this->overWriteExistingSpace= false;
    planningSteps          = 0;

    if(connNodesEnabled)
        planningSteps|=NODES_CONNECT;
    if(regGridEnabled)
        planningSteps|=REGULAR_GRID;
    if(expObstEnabled)
        planningSteps|=OBST_EXPAND;
    if(obstPenEnabled)
        planningSteps|=OBST_PENALTY;
    if(bridgeTestEnabled)
        planningSteps|=BRIDGE_TEST;

    this->executionStep = WAITING;
    robotManager->robot->setCheckPoints(obst_exp);
    connect(this, SIGNAL(addMsg(int,int,QString)), robMan->playGround,SLOT(addMsg(int,int,QString)));
    this->setupPlanner();
}

PlanningManager::PlanningManager(RobotManager *robMan):
pathPlanner(NULL),
renderSearchSpaceTree(false),
renderSearchTree(false),
renderPaths(false),
robotManager(robMan),
loadSpaceFromFile(true),
overWriteExistingSpace(false),
planningSteps(0),
executionStep(WAITING)
{
    this->connNodesEnabled      = CasPlanner::settings().isConnectNodesEnabled();
    this->regGridEnabled        = CasPlanner::settings().isRegGridEnabled();
    this->obstPenEnabled        = CasPlanner::settings().isObstaclePenEnabled();
    this->expObstEnabled        = CasPlanner::settings().isExpandObstEnabled();
    this->bridgeTestEnabled     = CasPlanner::settings().isBridgeTestEnabled();
    connect(this, SIGNAL(addMsg(int,int,QString)), robMan->playGround,SLOT(addMsg(int,int,QString)));
    if(connNodesEnabled)
        planningSteps|=NODES_CONNECT;
    if(regGridEnabled)
        planningSteps|=REGULAR_GRID;
    if(expObstEnabled)
        planningSteps|=OBST_EXPAND;
    if(obstPenEnabled)
        planningSteps|=OBST_PENALTY;
    if(bridgeTestEnabled)
        planningSteps|=BRIDGE_TEST;
}

PlanningManager::~PlanningManager()
{

}

void PlanningManager::setRobotManager(RobotManager *rob)
{
    this->robotManager = rob;
}

void PlanningManager:: setBridgeTest(bool state)
{
    bridgeTestEnabled = state;
    if(state)
        planningSteps^=BRIDGE_TEST;
    else
        planningSteps&=(0x1F^BRIDGE_TEST);
    LOG(Logger::Info,"PlanningSteps:"<<planningSteps)
}

void PlanningManager:: setConnNodes(bool state)
{
    connNodesEnabled = state;
    if(state)
        planningSteps^=NODES_CONNECT;
    else
        planningSteps&=(0x1F^NODES_CONNECT);
    LOG(Logger::Info,"PlanningSteps:"<<planningSteps)
}

void PlanningManager:: setRegGrid(bool state)
{
    regGridEnabled = state;
    if(state)
        planningSteps^=REGULAR_GRID;
    else
        planningSteps&=(0x1F^REGULAR_GRID);
    LOG(Logger::Info,"PlanningSteps:"<<planningSteps)
}

void PlanningManager:: setObstPen(bool state)
{
    obstPenEnabled = state;
    if(state)
        planningSteps^=OBST_PENALTY;
    else
        planningSteps&=(0x1F^OBST_PENALTY);
    LOG(Logger::Info,"PlanningSteps:"<<planningSteps)
}

void PlanningManager:: setExpObst(bool state)
{
    expObstEnabled = state;
    if(state)
        planningSteps^=OBST_EXPAND;
    else
        planningSteps&=(0x1F^OBST_EXPAND);
    LOG(Logger::Info,"PlanningSteps:"<<planningSteps)
}

void PlanningManager::setBridgeTestValue(double val)
{
    pathPlanner->setBridgeLen(val);
}

void PlanningManager::setConnNodesValue(double val )
{
    pathPlanner->setConRad(val);
}

void PlanningManager::setRegGridValue(double val)
{
    pathPlanner->setRegGrid(val);
}

void PlanningManager::setObstPenValue(double val)
{
    pathPlanner->setObstDist(val);
}

void PlanningManager::setExpObstValue(double val)
{
    pathPlanner->setExpRad(val);
}

void PlanningManager::setBridgeResValue(double val)
{
    pathPlanner->setBridgeRes(val);
}

void PlanningManager::setMap(Map * mapData)
{
    if(!this->pathPlanner)
        this->setupPlanner();
    pathPlanner->setMap(mapData);
}

void PlanningManager::setMap(LaserScan laserScan,double local_dist,Pose robotLocation)
{
    if(!this->pathPlanner)
        this->setupPlanner();
    pathPlanner->setMap(robotManager->playGround->mapManager->provideLaserOG(laserScan,local_dist,pixel_res,robotLocation));
}

void PlanningManager::updateMap(LaserScan laserScan,double local_dist,Pose robotLocation)
{
    Map *newMap = robotManager->playGround->mapManager->providePointCloud(laserScan,local_dist,robotLocation);
    if(!this->pathPlanner)
        this->setupPlanner();
    pathPlanner->updateMap(newMap);
}

void PlanningManager::setStart(Pose start)
{
    this->start = start;
}

void PlanningManager::setEnd(Pose end)
{
    this->end = end;
}

bool PlanningManager::fileExist(const char * fname)
{
    struct stat stat_buf;
    if (stat(fname,&stat_buf) != 0)
        return false;
    return (stat_buf.st_mode & S_IFMT) == S_IFREG;
}

void PlanningManager::generateSpace()
{
    executionStep = GENERATING_SPACE;
    QThread::start();
}

void PlanningManager::findPath(int coord)
{
    this->coord  = coord;
    executionStep = FINDING_PATH;
    QThread::start();
}

void PlanningManager::loadPlanningParameters()
{

    bridge_len          = CasPlanner::settings().bridgeLength();
    bridge_res          = CasPlanner::settings().bridgeTestRes();
    reg_grid            = CasPlanner::settings().regGridRes();
    obst_exp            = CasPlanner::settings().obstacleExpandR();
    reg_grid_conn_rad   = CasPlanner::settings().nodeConnectionR();
    obst_pen            = CasPlanner::settings().obstaclePenR();
    dist_goal           = CasPlanner::settings().dist2Goal();
    bridge_conn_rad     = CasPlanner::settings().bridgeConnectionR();

    pixel_res           = CasPlanner::settings().mapRes();
    negate              = CasPlanner::settings().isMapPixelsNegated();

    robotManager->robot->setCheckPoints(obst_exp);
    this->setupPlanner();
}

int PlanningManager::readConfigs( ConfigFile *cf)
{
    int numSec;
    numSec = cf->GetSectionCount();
    for(int i=0; i < numSec; i++)
    {
        QString sectionName = cf->GetSectionType(i);
        if(sectionName == "Planner")
        {
            bridge_len =			cf->ReadFloat(i, "bridge_len",2);
            bridge_res = 			cf->ReadFloat(i, "bridge_res",0.5);
            reg_grid =				cf->ReadFloat(i, "reg_grid",0.5);
            obst_exp = 				cf->ReadFloat(i, "obst_exp",0.2);
            reg_grid_conn_rad =                 cf->ReadFloat(i, "reg_grid_conn_rad",0.8);
            obst_pen = 				cf->ReadFloat(i, "obst_pen",3);
            dist_goal = 			cf->ReadFloat(i, "dist_goal",0.2);
            bridge_conn_rad =                   cf->ReadFloat(i, "bridge_conn_rad",0.5);
        }
        if(sectionName == "Map")
        {
            pixel_res =  			cf->ReadFloat(i, "pixel_res",0.05);
            negate = 				cf->ReadInt(i, "negate",0);
        }
    }
    robotManager->robot->setCheckPoints(obst_exp);
    this->setupPlanner();
    return 1;
}

int PlanningManager::setupPlanner()
{
    QString logMsg;
    if (!pathPlanner)
    {
        logMsg.append("\n-> Starting Planner.");
        logMsg.append("\n\tPlanning Parameters:");
        logMsg.append(QString("\n\t\t\t Pixel Resolution = %1").arg(pixel_res));
        logMsg.append(QString("\n\t\t\t Distance to Goal = %1").arg(dist_goal));
        logMsg.append(QString("\n\t\t\t Bridge Test Lenght = %1").arg(bridge_len));
        logMsg.append(QString("\n\t\t\t Bridge Test Res = %1").arg(bridge_res));
        logMsg.append(QString("\n\t\t\t Bridge Conn Rad = %1").arg(bridge_conn_rad));
        logMsg.append(QString("\n\t\t\t Reg Grid Res  = %1").arg(reg_grid));
        logMsg.append(QString("\n\t\t\t Obstacle Expansion Radius = %1").arg(obst_exp));
        logMsg.append(QString("\n\t\t\t Connection Radius = %1").arg(reg_grid_conn_rad));
        logMsg.append(QString("\n\t\t\t Obstacle Penalty = %1").arg(obst_pen));

        pathPlanner = new PathPlanner(robotManager->robot,
                                      dist_goal,
                                      bridge_len,
                                      bridge_res,
                                      reg_grid,
                                      reg_grid_conn_rad,
                                      obst_pen,
                                      bridge_conn_rad);
        logMsg.append("\n->Planner Started.");
        LOG(Logger::Info,logMsg)
        Q_EMIT addMsg(0,INFO,logMsg);
    }
    return 1;
}

void PlanningManager::generateSearchSpace(bool loadFromFile,bool overWriteCurrent)
{
    loadSpaceFromFile        = loadFromFile;
    overWriteExistingSpace   = overWriteCurrent;
    executionStep             = GENERATING_SPACE;
    if(!this->isRunning())
    {
        QThread::start();
    }
}

void PlanningManager::generateSearchSpaceState(bool loadFromFile,bool overWriteCurrent)
{
    QTime timer;
    const char * filename = "logs/SearchSpace.txt";
    if(!this->pathPlanner)
        this->setupPlanner();
    if(pathPlanner->search_space && !overWriteCurrent && planningSteps==pathPlanner->getPlanningSteps())
    {
        LOG(Logger::Warning,"Search Space already Exist")
        return;
    }
    if(overWriteCurrent)
    {
        pathPlanner->freeResources();
    }
    timer.restart();
    bool generateNewSearchSpace = true;
    if(fileExist(filename) && loadFromFile)
    {
        LOG(Logger::Info,"Loading Space From file ...")
        if(pathPlanner->readSpaceFromFile(filename,planningSteps))
        {
            if(expObstEnabled)
                pathPlanner->expandObstacles();
            if(connNodesEnabled)
                pathPlanner->connectNodes();
            generateNewSearchSpace = false;
            LOG(Logger::Info,"File loading took:"<<timer.elapsed()/double(1000.00)<<" secs")                   
        }
        else
        {
            LOG(Logger::Warning,"Could not Load Search Space from File, or meta-data mismatch")
        }
    }
    // We come here only if the file doesn't exist, or when we fail to load a file
    // with similar planning parameters and steps
    if(generateNewSearchSpace)
    {
        LOG(Logger::Info,"Generating Space ...")
        LOG(Logger::Info,"PlanningSteps:"<<planningSteps)
        if(planningSteps & OBST_EXPAND)
            pathPlanner->expandObstacles();
        if(planningSteps & REGULAR_GRID)
            pathPlanner->generateRegularGrid();
        if(planningSteps & BRIDGE_TEST)
            pathPlanner->bridgeTest();
        if(planningSteps & OBST_PENALTY)
            pathPlanner->addCostToNodes();
        if(planningSteps & NODES_CONNECT)
            pathPlanner->connectNodes();
        pathPlanner->saveSpace2File(filename);
        Q_EMIT searchSpaceGenerated();
        LOG(Logger::Info,"Space Generation took:"<<timer.elapsed()/double(1000.00)<<" secs")
    }
    pathPlanner->showConnections();
    this->loadSpaceFromFile     = true;
    this->overWriteExistingSpace= false;
}

void PlanningManager::findPathState()
{
    if(!this->pathPlanner)
        this->setupPlanner();
    Node * retval;
    if(!pathPlanner->search_space || planningSteps!=pathPlanner->getPlanningSteps())
    {
        generateSearchSpaceState();
    }
    QTime timer;
    timer.restart();
    retval = pathPlanner->startSearch(start,end,coord);
    LOG(Logger::Info,"Path Finding took:"<<(timer.elapsed()/double(1000.00))<<" secs")
    Q_EMIT pathFound(retval);
    if(retval)
    {
        //pathPlanner->printNodeList();
    }
    else
    {
        LOG(Logger::Info,"No Path Found")
    }
}

void PlanningManager::run()
{
    switch(executionStep)
    {
    case GENERATING_SPACE:
        LOG(Logger::Info,"PlanningManager Thread is performing Space Generation State")
        generateSearchSpaceState(loadSpaceFromFile,overWriteExistingSpace);
        break;
    case FINDING_PATH:
        LOG(Logger::Info,"PlanningManager Thread is performing Path Finding State")
        findPathState();
        break;
    case WAITING:
        break;
    }
}

int PlanningManager::stop()
{
    return 1;
}

