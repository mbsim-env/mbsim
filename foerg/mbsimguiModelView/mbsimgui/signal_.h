/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _SIGNAL__H_
#define _SIGNAL__H_

#include "link.h"

class Signal : public Link {
  public:
    Signal(const QString &str, QTreeWidgetItem *parentItem, int ind);
    ~Signal(); 
    virtual QString getType() const { return "Signal"; }
};

class Sensor : public Signal {
  public:
    Sensor(const QString &str, QTreeWidgetItem *parentItem, int ind);
    ~Sensor(); 
    virtual QString getType() const { return "Sensor"; }
};

class AbsolutCoordinateSensor : public Sensor {
  public:
    AbsolutCoordinateSensor(const QString &str, QTreeWidgetItem *parentItem, int ind); 
    virtual QString getType() const { return "AbsolutCoordinateSensor"; }
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    ExtWidget *frame, *direction;
    //std::string frameString;
};

class AbsolutePositionSensor : public AbsolutCoordinateSensor {
  public:
    AbsolutePositionSensor(const QString &str, QTreeWidgetItem *parentItem, int ind); 
    virtual QString getType() const { return "AbsolutePositionSensor"; }
};


#endif
