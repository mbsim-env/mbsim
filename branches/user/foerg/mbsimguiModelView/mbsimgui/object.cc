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
#include "string_widgets.h"
#include "extended_widgets.h"
#include <QtGui/QMenu>

using namespace std;

Object::Object(const QString &str, QTreeWidgetItem *parentItem, int ind) : Element(str, parentItem, ind), q0(0), u0(0), q0Property(0,false), u0Property(0,false) {

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

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMNS"initialGeneralizedPosition"));
  q0Property.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  u0 = new VecWidget(0);
  input.push_back(new PhysicalStringProperty(new VecProperty(0),"",MBSIMNS"initialGeneralizedVelocity"));
  u0Property.setProperty(new ExtPhysicalVarProperty(input));
}

Object::~Object() {
}

void Object::initializeDialog() {
  Element::initializeDialog();
  dialog->addTab("Initial conditions");
  vector<PhysicalStringWidget*> input;
  q0 = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(q0,QStringList(),1));
  ExtPhysicalVarWidget *var = new ExtPhysicalVarWidget(input);  
  q0Widget = new ExtWidget("Initial generalized position",var,true);
  dialog->addToTab("Initial conditions", q0Widget);

  input.clear();
  u0 = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(u0,QStringList(),1));
  var = new ExtPhysicalVarWidget(input);  
  u0Widget = new ExtWidget("Initial generalized velocity",var,true);
  dialog->addToTab("Initial conditions", u0Widget);
}

void Object::toWidget() {
  Element::toWidget();
  q0Property.toWidget(q0Widget);
  u0Property.toWidget(u0Widget);
}

void Object::fromWidget() {
  Element::fromWidget();
  q0Property.fromWidget(q0Widget);
  u0Property.fromWidget(u0Widget);
}

void Object::initializeUsingXML(TiXmlElement *element) {
  Element::initializeUsingXML(element);
  q0Property.initializeUsingXML(element);
  u0Property.initializeUsingXML(element);
}

TiXmlElement* Object::writeXMLFile(TiXmlNode *parent) {    
  TiXmlElement *ele0 = Element::writeXMLFile(parent);
  q0Property.writeXMLFile(ele0);
  u0Property.writeXMLFile(ele0);
  return ele0;
}

Element * Object::getByPathSearch(QString path) {
  if (path.mid(0, 1)=="/") // absolut path
    if(getParentElement())
      return getParentElement()->getByPathSearch(path);
    else
      return getByPathSearch(path.mid(1));
  else if (path.mid(0, 3)=="../") // relative path
    return getParentElement()->getByPathSearch(path.mid(3));
  else  // local path
    throw MBSimError("Unknown identifier of container");
}

