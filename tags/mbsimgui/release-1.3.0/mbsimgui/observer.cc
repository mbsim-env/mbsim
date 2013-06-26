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
#include "objectfactory.h"

using namespace std;
using namespace MBXMLUtils;

Observer::Observer(const string &str, Element *parent) : Element(str, parent) {
}

Observer* Observer::readXMLFile(const string &filename, Element *parent) {
  TiXmlDocument doc;
  if(doc.LoadFile(filename)) {
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    Observer *observer=ObjectFactory::getInstance()->createObserver(e,parent);
    if(observer) {
      observer->initializeUsingXML(e);
      observer->initialize();
    }
    return observer;
  }
  return 0;
}

Element * Observer::getByPathSearch(string path) {
  if (path.substr(0, 3)=="../") // relative path
    return getParent()->getByPathSearch(path.substr(3));
  else // absolut path
    if(getParent())
      return getParent()->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  return NULL;
}

CoordinatesObserver::CoordinatesObserver(const string &str, Element *parent) : Observer(str, parent), position(0,false), velocity(0,false), acceleration(0,false) {

  frame.setProperty(new FrameOfReferenceProperty("",this,MBSIMNS"frame"));

  position.setProperty(new OMBVArrowProperty("NOTSET",getID(),true));
  position.setXMLName(MBSIMNS"openMBVPositionArrow",false);

  velocity.setProperty(new OMBVArrowProperty("NOTSET",getID(),true));
  velocity.setXMLName(MBSIMNS"openMBVVelocityArrow",false);

  acceleration.setProperty(new OMBVArrowProperty("NOTSET",getID(),true));
  acceleration.setXMLName(MBSIMNS"openMBVAccelerationArrow",false);
}

void CoordinatesObserver::initialize() {
  Observer::initialize();
  frame.initialize();
}

void CoordinatesObserver::initializeUsingXML(TiXmlElement *element) {
  Observer::initializeUsingXML(element);
  frame.initializeUsingXML(element);
  position.initializeUsingXML(element);
  velocity.initializeUsingXML(element);
  acceleration.initializeUsingXML(element);
}

TiXmlElement* CoordinatesObserver::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Observer::writeXMLFile(parent);
  frame.writeXMLFile(ele0);
  position.writeXMLFile(ele0);
  velocity.writeXMLFile(ele0);
  acceleration.writeXMLFile(ele0);
  return ele0;
}

KinematicsObserver::KinematicsObserver(const string &str, Element *parent) : Observer(str, parent), position(0,false), velocity(0,false), angularVelocity(0,false), acceleration(0,false), angularAcceleration(0,false) {

  frame.setProperty(new FrameOfReferenceProperty("",this,MBSIMNS"frame"));

  position.setProperty(new OMBVArrowProperty("NOTSET",getID(),true));
  position.setXMLName(MBSIMNS"openMBVPositionArrow",false);

  velocity.setProperty(new OMBVArrowProperty("NOTSET",getID(),true));
  velocity.setXMLName(MBSIMNS"openMBVVelocityArrow",false);

  angularVelocity.setProperty(new OMBVArrowProperty("NOTSET",getID(),true));
  angularVelocity.setXMLName(MBSIMNS"openMBVAngularVelocityArrow",false);

  acceleration.setProperty(new OMBVArrowProperty("NOTSET",getID(),true));
  acceleration.setXMLName(MBSIMNS"openMBVAccelerationArrow",false);

  angularAcceleration.setProperty(new OMBVArrowProperty("NOTSET",getID(),true));
  angularAcceleration.setXMLName(MBSIMNS"openMBVAngularAccelerationArrow",false);
}

void KinematicsObserver::initialize() {
  Observer::initialize();
  frame.initialize();
}

void KinematicsObserver::initializeUsingXML(TiXmlElement *element) {
  Observer::initializeUsingXML(element);
  frame.initializeUsingXML(element);
  position.initializeUsingXML(element);
  velocity.initializeUsingXML(element);
  angularVelocity.initializeUsingXML(element);
  acceleration.initializeUsingXML(element);
  angularAcceleration.initializeUsingXML(element);
}

TiXmlElement* KinematicsObserver::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Observer::writeXMLFile(parent);
  frame.writeXMLFile(ele0);
  position.writeXMLFile(ele0);
  velocity.writeXMLFile(ele0);
  angularVelocity.writeXMLFile(ele0);
  acceleration.writeXMLFile(ele0);
  angularAcceleration.writeXMLFile(ele0);
  return ele0;
}

RelativeKinematicsObserver::RelativeKinematicsObserver(const string &str, Element *parent) : KinematicsObserver(str, parent) {

  refFrame.setProperty(new FrameOfReferenceProperty("",this,MBSIMNS"frameOfReference"));
}

void RelativeKinematicsObserver::initialize() {
  KinematicsObserver::initialize();
  refFrame.initialize();
}

void RelativeKinematicsObserver::initializeUsingXML(TiXmlElement *element) {
  KinematicsObserver::initializeUsingXML(element);
  refFrame.initializeUsingXML(element);
}

TiXmlElement* RelativeKinematicsObserver::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = KinematicsObserver::writeXMLFile(parent);
  refFrame.writeXMLFile(ele0);
  return ele0;
}


