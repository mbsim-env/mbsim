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
#include "basic_widgets.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include "ombv_widgets.h"
#include <QtGui/QMenu>
#include "mainwindow.h"

using namespace std;

Observer::Observer(const QString &str, QTreeWidgetItem *parentItem, int ind) : Element(str, parentItem, ind) {

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Remove", this);
  connect(action,SIGNAL(triggered()),this,SLOT(remove()));
  contextMenu->addAction(action);
}

Observer::~Observer() {
}

Element * Observer::getByPathSearch(QString path) {
  if (path.mid(0, 3)=="../") // relative path
    return getParentElement()->getByPathSearch(path.mid(3));
  else // absolut path
    if(getParentElement())
      return getParentElement()->getByPathSearch(path);
    else
      return getByPathSearch(path.mid(1));
}

AbsoluteKinematicsObserver::AbsoluteKinematicsObserver(const QString &str, QTreeWidgetItem *parentItem, int ind) : Observer(str, parentItem, ind), position(0,false), velocity(0,false), angularVelocity(0,false), acceleration(0,false), angularAcceleration(0,false) {

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

void AbsoluteKinematicsObserver::initializeDialog() {
  Observer::initializeDialog();

  dialog->addTab("Visualisation");

  frameWidget = new ExtWidget("Frame",new FrameOfReferenceWidget(this,0));
  dialog->addToTab("General", frameWidget);

  positionWidget = new ExtWidget("OpenMBV position arrow",new OMBVArrowWidget("NOTSET",true),true);
  dialog->addToTab("Visualisation",positionWidget);

  velocityWidget = new ExtWidget("OpenMBV velocity arrow",new OMBVArrowWidget("NOTSET",true),true);
  dialog->addToTab("Visualisation",velocityWidget);

  angularVelocityWidget = new ExtWidget("OpenMBV angular velocity arrow",new OMBVArrowWidget("NOTSET",true),true);
  dialog->addToTab("Visualisation",angularVelocityWidget);

  accelerationWidget = new ExtWidget("OpenMBV acceleration arrow",new OMBVArrowWidget("NOTSET",true),true);
  dialog->addToTab("Visualisation",accelerationWidget);

  angularAccelerationWidget = new ExtWidget("OpenMBV angular acceleration arrow",new OMBVArrowWidget("NOTSET",true),true);
  dialog->addToTab("Visualisation",angularAccelerationWidget);
}

void AbsoluteKinematicsObserver::toWidget() {
  Observer::toWidget();
  frame.toWidget(frameWidget);
  position.toWidget(positionWidget);
  velocity.toWidget(velocityWidget);
  angularVelocity.toWidget(angularVelocityWidget);
  acceleration.toWidget(accelerationWidget);
  angularAcceleration.toWidget(angularAccelerationWidget);
}

void AbsoluteKinematicsObserver::fromWidget() {
  Observer::fromWidget();
  frame.fromWidget(frameWidget);
  position.fromWidget(positionWidget);
  velocity.fromWidget(velocityWidget);
  angularVelocity.fromWidget(angularVelocityWidget);
  acceleration.fromWidget(accelerationWidget);
  angularAcceleration.fromWidget(angularAccelerationWidget);
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
