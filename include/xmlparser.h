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
#ifndef XMLPARSER_H_
#define XMLPARSER_H_

#include <QObject>
#include <QFile> 
#include <QString>
#include <QDomDocument>
#include <iostream>
#include <QDomNamedNodeMap>
#include <QDomNode>
#include <QDomAttr>

class XmlParser: public QObject
{
	Q_OBJECT 
	public:
		XmlParser();
		XmlParser(QFile *file);	
		XmlParser(QString fileName);
		virtual ~XmlParser();
	    bool read(QIODevice *device);
	    void read();
	    bool write(QIODevice *device);	
	private:
	    QDomDocument configFileDom;
	    QFile *configFile;
	    QString configFileName;
};

#endif /*XMLPARSER_H_*/
