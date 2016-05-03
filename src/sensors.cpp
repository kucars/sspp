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
#include "sensors.h"

Sensors::Sensors(double hfov, double vfov, double fl, double nearR, double farR, double width, double height, Vec3f sP, Vec3f sRPY):
hFOV(hfov),
vFOV(vfov),
focalLength(fl),
near(nearR),
far(farR),
resWidth(width),
resHeight(height),
sensorPose(sP),
sensorRPY(sRPY)
{

}


Sensors::Sensors()
{
}

Sensors::~Sensors()
{
}

geometry_msgs::Pose Sensors::robot2sensorTransformation(geometry_msgs::Pose pose)
{


    Eigen::Matrix4d robotPoseMat, robot2sensorMat, sensorPoseMat;
    //Robot matrix pose
    Eigen::Matrix3d R; Eigen::Vector3d T1(pose.position.x,pose.position.y,pose.position.z);
    tf::Quaternion qt(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    tf::Matrix3x3 R1(qt);
    tf::matrixTFToEigen(R1,R);
    robotPoseMat.setZero ();
    robotPoseMat.block (0, 0, 3, 3) = R;
    robotPoseMat.block (0, 3, 3, 1) = T1;
    robotPoseMat (3, 3) = 1;

    //transformation matrix
    qt = tf::createQuaternionFromRPY(sensorRPY[0],sensorRPY[1],sensorRPY[2]);
    tf::Matrix3x3 R2(qt);Eigen::Vector3d T2(sensorPose[0], sensorPose[1], sensorPose[2]);
    tf::matrixTFToEigen(R2,R);
    robot2sensorMat.setZero ();
    robot2sensorMat.block (0, 0, 3, 3) = R;
    robot2sensorMat.block (0, 3, 3, 1) = T2;
    robot2sensorMat (3, 3) = 1;

    //preform the transformation
    sensorPoseMat = robotPoseMat * robot2sensorMat;

    Eigen::Matrix4d sensor2sensorMat; //the frustum culling sensor needs this
    //the transofrmation is rotation by +90 around x axis of the sensor
    sensor2sensorMat << 1, 0, 0, 0,
                        0, 0,-1, 0,
                        0, 1, 0, 0,
                        0, 0, 0, 1;
    Eigen::Matrix4d newSensorPoseMat = sensorPoseMat * sensor2sensorMat;
    geometry_msgs::Pose p;
    Eigen::Vector3d T3;Eigen::Matrix3d Rd; tf::Matrix3x3 R3;
    Rd = newSensorPoseMat.block (0, 0, 3, 3);
    tf::matrixEigenToTF(Rd,R3);
    T3 = newSensorPoseMat.block (0, 3, 3, 1);
    p.position.x=T3[0];p.position.y=T3[1];p.position.z=T3[2];
    R3.getRotation(qt);
    p.orientation.x = qt.getX(); p.orientation.y = qt.getY();p.orientation.z = qt.getZ();p.orientation.w = qt.getW();

    return p;

}
