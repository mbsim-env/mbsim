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
#include "object.h"
#include "utils.h"
#include <QtGui/QMenu>

using namespace std;


Object::Object(const QString &str, QTreeWidgetItem *parentItem, int ind) : Element(str, parentItem, ind) {
  properties->addTab("Initial conditions");

  vector<PhysicalStringWidget*> input;
  q0 = new SVecWidget(0);
  input.push_back(new PhysicalStringWidget(q0,MBSIMNS"initialGeneralizedPosition",QStringList(),1));
  ExtPhysicalVarWidget *var = new ExtPhysicalVarWidget(input);  
  initialGeneralizedPosition = new ExtXMLWidget("Initial generalized position",var,true);
  properties->addToTab("Initial conditions", initialGeneralizedPosition);
  connect(initialGeneralizedPosition,SIGNAL(resize()),this,SLOT(resizeGeneralizedPosition()));

  input.clear();
  u0 = new SVecWidget(0);
  input.push_back(new PhysicalStringWidget(u0,MBSIMNS"initialGeneralizedVelocity",QStringList(),1));
  var = new ExtPhysicalVarWidget(input);  
  initialGeneralizedVelocity = new ExtXMLWidget("Initial generalized velocity",var,true);
  properties->addToTab("Initial conditions", initialGeneralizedVelocity);
  connect(initialGeneralizedVelocity,SIGNAL(resize()),this,SLOT(resizeGeneralizedVelocity()));

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

  action=new QAction(Utils::QIconCached("newobject.svg"),"Remove", this);
  connect(action,SIGNAL(triggered()),this,SLOT(remove()));
  contextMenu->addAction(action);

  contextMenu->addSeparator();

}

Object::~Object() {
}

void Object::update() {
  Element::update();
}

void Object::initializeUsingXML(TiXmlElement *element) {
  Element::initializeUsingXML(element);
  initialGeneralizedPosition->initializeUsingXML(element);
  initialGeneralizedVelocity->initializeUsingXML(element);
}

TiXmlElement* Object::writeXMLFile(TiXmlNode *parent) {    
  TiXmlElement *ele0 = Element::writeXMLFile(parent);
  initialGeneralizedPosition->writeXMLFile(ele0);
  initialGeneralizedVelocity->writeXMLFile(ele0);
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

