#ifndef SETTINGS_H
#define SETTINGS_H

#include <QMap>
#include <QPoint>
#include <QSettings>
#include <QSize>
#include <QStringList>
#include <QMutex>
#include <QMutexLocker>
#include <QCoreApplication>

class Planning : public QSettings
{
public:
    Planning()
    {
        beginGroup( "Planning" );
    }
};


class DisplaySettings : public QSettings
{
public:
    DisplaySettings()
    {
        beginGroup( "Display" );
    }
};

class CasPlannerSettings : public QSettings
{
    Q_OBJECT
 public:
    CasPlannerSettings( QObject* parent );
    void setCurrentUsername( QString username );

    double mapRes()               { return Planning().value( "MapResolution",0.047).toDouble(); }
    void setMapRes(double dist)   { Planning().setValue("MapResolution", dist ); }

    bool isMapPixelsNegated()           { return DisplaySettings().value( "MapPixelsNegated", 0 ).toBool(); }
    void setMapPixelsNegated(bool ena)  { DisplaySettings().setValue("MapPixelsNegated", ena ); }

    double dist2Goal()               { return Planning().value( "Distance2Goal",0.2).toDouble(); }
    void setdist2Goal(double dist)   { Planning().setValue("Distance2Goal", dist ); }

    double bridgeTestRes()              { return Planning().value( "BridgeTestRes", 0.05 ).toDouble(); }
    void setBridgeTestRes(double res)   { Planning().setValue("BridgeTestRes", res ); }

    double bridgeLength()               { return Planning().value( "BridgeLength", 2.5 ).toDouble(); }
    void setBridgeLength(double len)    { Planning().setValue("BridgeLength", len ); }

    double regGridRes()                 { return Planning().value( "RegularGridRes", 0.2 ).toDouble(); }
    void setRegGridRes(double res)      { Planning().setValue("RegularGridRes", res ); }

    double bridgeConnectionR()            { return Planning().value( "BridgeConnectionRadius", 0.5 ).toDouble(); }
    void setbridgeConnectionR(double rad) { Planning().setValue("BridgeConnectionRadius", rad ); }

    double nodeConnectionR()            { return Planning().value( "NodeConnectionRadius", 0.4 ).toDouble(); }
    void setNodeConnectionR(double rad) { Planning().setValue("NodeConnectionRadius", rad ); }

    double obstaclePenR()               { return Planning().value( "ObstaclePenaltyRadius", 3.0 ).toDouble(); }
    void setObstaclePenR(double rad)    { Planning().setValue("ObstaclePenaltyRadius", rad ); }

    double obstacleExpandR()            { return Planning().value( "ObstacleExpansionRadius", 0.4685 ).toDouble(); }
    void setObstacleExpandR(double rad) { Planning().setValue("ObstacleExpansionRadius", rad ); }

    int obstacleAvoidAlgo()             { return Planning().value( "ObstacleAvoidanceAlgo", 1 ).toInt(); }
    void setObstacleAvoidAlgo(int algo) { Planning().setValue("ObstacleAvoidanceAlgo", algo ); }

    bool isBridgeTestEnabled()          { return Planning().value( "BridgeTestEnabled", 1 ).toBool(); }
    void setBridgeTest(bool ena)        { Planning().setValue("BridgeTestEnabled", ena ); }

    bool isConnectNodesEnabled()        { return Planning().value( "ConnectNodesEnabled", 1 ).toBool(); }
    void setConnectNodes(bool ena)      { Planning().setValue("ConnectNodesEnabled", ena ); }

    bool isRegGridEnabled()             { return Planning().value( "RegGridEnabled", 1 ).toBool(); }
    void setRegGrid(bool ena)           { Planning().setValue("RegGridEnabled", ena ); }

    bool isObstaclePenEnabled()         { return Planning().value( "ObstaclePenaltyEnabled", 1 ).toBool(); }
    void setObstaclePen(bool ena)       { Planning().setValue("ObstaclePenaltyEnabled", ena ); }

    bool isExpandObstEnabled()          { return Planning().value( "ExpandObstaclesEnabled", 1 ).toBool(); }
    void setExpandObst(bool ena)        { Planning().setValue("ExpandObstaclesEnabled", ena ); }

    bool isShowSearchTreeEnabled()      { return DisplaySettings().value( "ShowSearchTreeEnabled", 1 ).toBool(); }
    void setShowSearchTree(bool ena)    { DisplaySettings().setValue("ShowSearchTreeEnabled", ena ); }

    bool isShowSearchSpaceTreeEnabled() { return DisplaySettings().value( "ShowSearchSpaceTreeEnabled", 1 ).toBool(); }
    void setShowSearchSpaceTree(bool en){ DisplaySettings().setValue("ShowSearchSpaceTreeEnabled", en ); }

    bool isShowPathsEnabled()           { return DisplaySettings().value( "ShowPathsEnabled", 1 ).toBool(); }
    void setShowPaths(bool ena)         { DisplaySettings().setValue("ShowPathsEnabled", ena ); }

    bool isShowRobotTrailEnabled()      { return DisplaySettings().value( "ShowRobotTrailEnabled", 1 ).toBool(); }
    void setShowRobotTrail(bool ena)    { DisplaySettings().setValue("ShowRobotTrailEnabled", ena ); }

    bool isShowSearchSpaceSamplesEnabled()  { return DisplaySettings().value( "ShowSearchSpaceSamplesEnabled", 1 ).toBool(); }
    void setShowSearchSpaceSamples(bool ena){ DisplaySettings().setValue("ShowSearchSpaceSamplesEnabled", ena ); }

};


namespace CasPlanner
{
    inline CasPlannerSettings &settings()
    {
        static QMutex mutex;
        static CasPlannerSettings* settings = 0;
        QMutexLocker locker( &mutex );
        if (!settings)
        {
            settings = QCoreApplication::instance()->findChild<CasPlannerSettings*>( "CasPlanner-Settings-Instance" );
            if (!settings)
            {
                settings = new CasPlannerSettings( QCoreApplication::instance());
                settings->setObjectName( "CasPlanner-Settings-Instance" );
            }
        }
        return *settings;
    }
}

#endif
