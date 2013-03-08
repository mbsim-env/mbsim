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

AbsoluteKinematicsObserver::AbsoluteKinematicsObserver(const QString &str, QTreeWidgetItem *parentItem, int ind) : Observer(str, parentItem, ind) {
  properties->addTab("Visualisation");

  frame = new ExtWidget("Frame",new FrameOfReferenceWidget(this,0));
  properties->addToTab("General", frame);

  position = new ExtWidget("OpenMBV position arrow",new OMBVArrowWidget("NOTSET",true),true);
  ((OMBVArrowWidget*)position->getWidget())->setID(getID());
  properties->addToTab("Visualisation",position);

  velocity = new ExtWidget("OpenMBV velocity arrow",new OMBVArrowWidget("NOTSET",true),true);
  ((OMBVArrowWidget*)velocity->getWidget())->setID(getID());
  properties->addToTab("Visualisation",velocity);

  angularVelocity = new ExtWidget("OpenMBV angular velocity arrow",new OMBVArrowWidget("NOTSET",true),true);
  ((OMBVArrowWidget*)angularVelocity->getWidget())->setID(getID());
  properties->addToTab("Visualisation",angularVelocity);

  acceleration = new ExtWidget("OpenMBV acceleration arrow",new OMBVArrowWidget("NOTSET",true),true);
  ((OMBVArrowWidget*)acceleration->getWidget())->setID(getID());
  properties->addToTab("Visualisation",acceleration);

  angularAcceleration = new ExtWidget("OpenMBV angular acceleration arrow",new OMBVArrowWidget("NOTSET",true),true);
  ((OMBVArrowWidget*)angularAcceleration->getWidget())->setID(getID());
  properties->addToTab("Visualisation",angularAcceleration);

  // vector<PhysicalStringWidget*> input;
  // input.push_back(new PhysicalStringWidget(new ScalarWidget("0.5"), MBSIMNS"diameter", lengthUnits(), 4));
  // diameter = new ExtWidget("Diameter",new ExtPhysicalVarWidget(input));
  // properties->addToTab("General", diameter);

  // input.clear();
  // input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), MBSIMNS"headDiameter", lengthUnits(), 1));
  // headDiameter = new ExtWidget("HeadDiameter",new ExtPhysicalVarWidget(input));
  // properties->addToTab("General", headDiameter);

  // input.clear();
  // input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), MBSIMNS"headLength", lengthUnits(), 1));
  // headLength = new ExtWidget("HeadLength",new ExtPhysicalVarWidget(input));
  // properties->addToTab("General", headLength);

  // input.clear();
  // input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), MBSIMNS"staticColor", noUnitUnits(), 1));
  // color = new ExtWidget("StaticColor",new ExtPhysicalVarWidget(input));
  // properties->addToTab("General", color);

  properties->addStretch();
}

AbsoluteKinematicsObserver::~AbsoluteKinematicsObserver() {
}

void AbsoluteKinematicsObserver::initializeUsingXML(TiXmlElement *element) {
  Observer::initializeUsingXML(element);
  frame->initializeUsingXML(element);
  //  TiXmlElement *e=element->FirstChildElement(MBSIMNS"enableOpenMBVPosition");
  //  if(e) {
  //    diameter->initializeUsingXML(element);
  //    headDiameter->initializeUsingXML(element);
  //    headLength->initializeUsingXML(element);
  //    color->initializeUsingXML(element);
  //  }
  position->initializeUsingXML(element);
  velocity->initializeUsingXML(element);
  angularVelocity->initializeUsingXML(element);
  acceleration->initializeUsingXML(element);
  angularAcceleration->initializeUsingXML(element);
}

TiXmlElement* AbsoluteKinematicsObserver::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Observer::writeXMLFile(parent);
  frame->writeXMLFile(ele0);
  position->writeXMLFile(ele0);
  velocity->writeXMLFile(ele0);
  angularVelocity->writeXMLFile(ele0);
  acceleration->writeXMLFile(ele0);
  angularAcceleration->writeXMLFile(ele0);
  //  TiXmlElement *ele1=new TiXmlElement(MBSIMNS"enableOpenMBVPosition");
  //  diameter->writeXMLFile(ele1);
  //  headDiameter->writeXMLFile(ele1);
  //  headLength->writeXMLFile(ele1);
  //  color->writeXMLFile(ele1);
  //  ele0->LinkEndChild(ele1);
  return ele0;
}
