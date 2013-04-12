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

#include <config.h>
#include "observer.h"
#include "basic_properties.h"
#include "ombv_properties.h"
#include <QtGui/QMenu>
#include "mainwindow.h"

using namespace std;

Observer::Observer(const string &str, Element *parent) : Element(str, parent) {
}

Observer::~Observer() {
}

Element * Observer::getByPathSearch(string path) {
  if (path.substr(0, 3)=="../") // relative path
    return getParent()->getByPathSearch(path.substr(3));
  else // absolut path
    if(getParent())
      return getParent()->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
}

AbsoluteKinematicsObserver::AbsoluteKinematicsObserver(const string &str, Element *parent) : Observer(str, parent), position(0,false), velocity(0,false), angularVelocity(0,false), acceleration(0,false), angularAcceleration(0,false) {

  frame.setProperty(new FrameOfReferenceProperty(0,this,MBSIMNS"frame"));

  position.setProperty(new OMBVArrowProperty("NOTSET",true));
  ((OMBVArrowProperty*)position.getProperty())->setID(getID());
  position.setXMLName(MBSIMNS"openMBVPositionArrow",false);

  velocity.setProperty(new OMBVArrowProperty("NOTSET",true));
  ((OMBVArrowProperty*)velocity.getProperty())->setID(getID());
  velocity.setXMLName(MBSIMNS"openMBVVelocityArrow",false);

  angularVelocity.setProperty(new OMBVArrowProperty("NOTSET",true));
  ((OMBVArrowProperty*)angularVelocity.getProperty())->setID(getID());
  angularVelocity.setXMLName(MBSIMNS"openMBVAngularVelocityArrow",false);

  acceleration.setProperty(new OMBVArrowProperty("NOTSET",true));
  ((OMBVArrowProperty*)acceleration.getProperty())->setID(getID());
  acceleration.setXMLName(MBSIMNS"openMBVAccelerationArrow",false);

  angularAcceleration.setProperty(new OMBVArrowProperty("NOTSET",true));
  ((OMBVArrowProperty*)angularAcceleration.getProperty())->setID(getID());
  angularAcceleration.setXMLName(MBSIMNS"openMBVAngularAccelerationArrow",false);
}

AbsoluteKinematicsObserver::~AbsoluteKinematicsObserver() {
}

void AbsoluteKinematicsObserver::initialize() {
  Observer::initialize();
  frame.initialize();
}

void AbsoluteKinematicsObserver::initializeUsingXML(TiXmlElement *element) {
  Observer::initializeUsingXML(element);
  frame.initializeUsingXML(element);
  //  TiXmlElement *e=element->FirstChildElement(MBSIMNS"enableOpenMBVPosition");
  //  if(e) {
  //    diameter->initializeUsingXML(element);
  //    headDiameter->initializeUsingXML(element);
  //    headLength->initializeUsingXML(element);
  //    color->initializeUsingXML(element);
  //  }
  position.initializeUsingXML(element);
  velocity.initializeUsingXML(element);
  angularVelocity.initializeUsingXML(element);
  acceleration.initializeUsingXML(element);
  angularAcceleration.initializeUsingXML(element);
}

TiXmlElement* AbsoluteKinematicsObserver::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Observer::writeXMLFile(parent);
  frame.writeXMLFile(ele0);
  position.writeXMLFile(ele0);
  velocity.writeXMLFile(ele0);
  angularVelocity.writeXMLFile(ele0);
  acceleration.writeXMLFile(ele0);
  angularAcceleration.writeXMLFile(ele0);
  //  TiXmlElement *ele1=new TiXmlElement(MBSIMNS"enableOpenMBVPosition");
  //  diameter->writeXMLFile(ele1);
  //  headDiameter->writeXMLFile(ele1);
  //  headLength->writeXMLFile(ele1);
  //  color->writeXMLFile(ele1);
  //  ele0->LinkEndChild(ele1);
  return ele0;
}
