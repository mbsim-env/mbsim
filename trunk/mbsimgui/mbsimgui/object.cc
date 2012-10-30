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

#include "object.h"
#include "utils.h"
#include <QtGui/QMenu>

using namespace std;


Object::Object(const QString &str, QTreeWidgetItem *parentItem, int ind) : Element(str, parentItem, ind) {
  properties->addTab("Initial conditions");
  initialGeneralizedPosition=new GeneralizedCoordinatesEditor(properties, Utils::QIconCached("lines.svg"), "Initial generalized position", "Initial conditions", MBSIMNS"initialGeneralizedPosition");
  initialGeneralizedVelocity=new GeneralizedCoordinatesEditor(properties, Utils::QIconCached("lines.svg"), "Initial generalized velocity", "Initial conditions", MBSIMNS"initialGeneralizedVelocity");
  connect(initialGeneralizedPosition,SIGNAL(resizeGeneralizedCoordinates()),this,SLOT(resizeGeneralizedPosition()));
  connect(initialGeneralizedPosition,SIGNAL(disableGeneralizedCoordinates()),this,SLOT(disableGeneralizedPosition()));
  connect(initialGeneralizedVelocity,SIGNAL(resizeGeneralizedCoordinates()),this,SLOT(resizeGeneralizedVelocity()));
  connect(initialGeneralizedVelocity,SIGNAL(disableGeneralizedCoordinates()),this,SLOT(disableGeneralizedVelocity()));

  actionSaveAs=new QAction(Utils::QIconCached("newobject.svg"),"Save as", this);
  connect(actionSaveAs,SIGNAL(triggered()),this,SLOT(saveAs()));
  contextMenu->addAction(actionSaveAs);

  actionSave=new QAction(Utils::QIconCached("newobject.svg"),"Save", this);
  actionSave->setDisabled(true);
  connect(actionSave,SIGNAL(triggered()),this,SLOT(save()));
  contextMenu->addAction(actionSave);

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Copy", this);
  connect(action,SIGNAL(triggered()),this,SLOT(copy()));
  contextMenu->addAction(action);

  contextMenu->addSeparator();

}

Object::~Object() {
}

void Object::update() {
  Element::update();
}

void Object::disableGeneralizedPosition() {
  ((SVecWidget*)initialGeneralizedPosition->getExtPhysicalWidget()->getPhysicalStringWidget(0)->getWidget())->resize(0);
}

void Object::disableGeneralizedVelocity() {
  ((SVecWidget*)initialGeneralizedVelocity->getExtPhysicalWidget()->getPhysicalStringWidget(0)->getWidget())->resize(0);
}


void Object::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Element::initializeUsingXML(element);
    initialGeneralizedPosition->initializeUsingXML(element);
    initialGeneralizedVelocity->initializeUsingXML(element);
//    e=element->FirstChildElement(MBSIMNS"initialGeneralizedPosition");
//    if (e)
//      initialGeneralizedPosition->setVec(getVec(e));
//    e=element->FirstChildElement(MBSIMNS"initialGeneralizedVelocity");
//    if (e)
//      initialGeneralizedVelocity->setVec(getVec(e));
}

TiXmlElement* Object::writeXMLFile(TiXmlNode *parent) {    
  TiXmlElement *ele0 = Element::writeXMLFile(parent);
  string str = (initialGeneralizedPosition->getExtPhysicalWidget())->getValue();
  if(str!="[]" && str!="")
    initialGeneralizedPosition->writeXMLFile(ele0);
  str = (initialGeneralizedVelocity->getExtPhysicalWidget())->getValue();
  if(str!="[]" && str!="")
    initialGeneralizedVelocity->writeXMLFile(ele0);

 // if(initialGeneralizedPosition->getRows()) 
 //   addElementText(ele0,MBSIMNS"initialGeneralizedPosition",initialGeneralizedPosition->getMat());
 // if(initialGeneralizedVelocity->getRows()) 
 //   addElementText(ele0,MBSIMNS"initialGeneralizedVelocity",initialGeneralizedVelocity->getMat());
  return ele0;
}

Element * Object::getByPathSearch(string path) {
  if (path.substr(0, 1)=="/") // absolut path
    if(getParentElement())
      return getParentElement()->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  else if (path.substr(0, 3)=="../") // relative path
    return getParentElement()->getByPathSearch(path.substr(3));
  else  // local path
    throw MBSimError("Unknown identifier of container");
}

