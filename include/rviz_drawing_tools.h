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
#ifndef RVIZ_DRAWING_TOOLS_H_
#define RVIZ_DRAWING_TOOLS_H_
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker drawLines(std::vector<geometry_msgs::Point> links, int id, int c_color, int duration, double scale);
visualization_msgs::Marker drawPoints(std::vector<geometry_msgs::Point> points, int c_color, int duration);

#endif //RVIZ_DRAWING_TOOLS_H_
