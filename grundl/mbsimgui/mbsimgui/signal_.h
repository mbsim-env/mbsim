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
#include "extended_properties.h"

class Signal : public Link {
  public:
    Signal(const std::string &str, Element *parent);
    virtual std::string getNameSpace() const { return MBSIMCONTROLNS; }
    ~Signal(); 
};

class Sensor : public Signal {
  public:
    Sensor(const std::string &str, Element *parent);
    ~Sensor(); 
};

class AbsoluteCoordinateSensor : public Sensor {
  friend class AbsoluteCoordinateSensorPropertyDialog;
  public:
    AbsoluteCoordinateSensor(const std::string &str, Element *parent);
    virtual std::string getType() const { return "AbsoluteCoordinateSensor"; }
    virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void initialize();
  protected:
    ExtProperty frame, direction;
};

class AbsolutePositionSensor : public AbsoluteCoordinateSensor {
  public:
    AbsolutePositionSensor(const std::string &str, Element *parent);
    virtual std::string getType() const { return "AbsolutePositionSensor"; }
    ElementPropertyDialog* createPropertyDialog() {return new AbsolutePositionSensorPropertyDialog(this);}
};


#endif
