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
#include "contour.h"
#include "basic_properties.h"
#include "ombv_properties.h"
#include "basic_widgets.h"
#include "string_widgets.h"
#include "ombv_widgets.h"
#include <QMenu>

using namespace std;

Contour::Contour(const QString &str, QTreeWidgetItem *parentItem, int ind) : Element(str,parentItem,ind), refFrame(0,false) {
  refFrame.setProperty(new ParentFrameOfReferenceProperty(0,this,MBSIMNS"frameOfReference"));
}

Contour::~Contour() {
}

void Contour::initialize() {
  Element::initialize();
  refFrame.initialize();
}

void Contour::initializeDialog() {
  Element::initializeDialog();
  refFrameWidget = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(this,0),true);
  dialog->addToTab("General", refFrameWidget);
}

void Contour::toWidget() {
  Element::toWidget();
  refFrame.toWidget(refFrameWidget);
}

void Contour::fromWidget() {
  Element::fromWidget();
  refFrame.fromWidget(refFrameWidget);
}

void Contour::setSavedFrameOfReference(const QString &str) {
  ((ParentFrameOfReferenceProperty*)(refFrame.getProperty()))->setSavedFrameOfReference(str);
}

void Contour::initializeUsingXML(TiXmlElement *element) {
  Element::initializeUsingXML(element);
  refFrame.initializeUsingXML(element);
}

TiXmlElement* Contour::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Element::writeXMLFile(parent);
  refFrame.writeXMLFile(ele0);
  return ele0;
}

Element *Contour::getByPathSearch(QString path) {
  if (path.mid(0, 1)=="/") // absolut path
    if(getParentElement())
      return getParentElement()->getByPathSearch(path);
    else
      return getByPathSearch(path.mid(1));
  else if (path.mid(0, 3)=="../") // relative path
    return getParentElement()->getByPathSearch(path.mid(3));
  else { // local path
    throw;
  }
}

Point::Point(const QString &str, QTreeWidgetItem *parentItem, int ind) : Contour(str,parentItem,ind) {
  setText(1,getType());
}

Point::~Point() {
}

Line::Line(const QString &str, QTreeWidgetItem *parentItem, int ind) : Contour(str,parentItem,ind) {
  setText(1,getType());
}

Line::~Line() {
}

Plane::Plane(const QString &str, QTreeWidgetItem *parentItem, int ind) : Contour(str,parentItem,ind), visu(0,false) {
  setText(1,getType());

  visu.setProperty(new OMBVPlaneProperty(MBSIMNS"enableOpenMBV"));
  ((OMBVPlaneProperty*)visu.getProperty())->setID(getID());
}

Plane::~Plane() {
}

void Plane::initializeDialog() {
  Contour::initializeDialog();

  dialog->addTab("Visualisation");

  visuWidget = new ExtWidget("OpenMBV Plane",new OMBVPlaneWidget,true);
  dialog->addToTab("Visualisation", visuWidget);
}

void Plane::initializeUsingXML(TiXmlElement *element) {
  Contour::initializeUsingXML(element);
  visu.initializeUsingXML(element);
}

TiXmlElement* Plane::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e = Contour::writeXMLFile(parent);
  visu.writeXMLFile(e);
  return e;
}

void Plane::toWidget() {
  Contour::toWidget();
  visu.toWidget(visuWidget);
}

void Plane::fromWidget() {
  Contour::fromWidget();
  visu.fromWidget(visuWidget);
}

Sphere::Sphere(const QString &str, QTreeWidgetItem *parentItem, int ind) : Contour(str,parentItem,ind), visu(0,false) {
  setText(1,getType());
 
  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1"), "m", MBSIMNS"radius"));
  radius.setProperty(new ExtPhysicalVarProperty(input));

  visu.setProperty(new OMBVEmptyProperty(MBSIMNS"enableOpenMBV"));
  ((OMBVEmptyProperty*)visu.getProperty())->setID(getID());

}

Sphere::~Sphere() {
}

void Sphere::initializeDialog() {
  Contour::initializeDialog();

  dialog->addTab("Visualisation");
 
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), lengthUnits(), 4));
  radiusWidget = new ExtWidget("Radius",new ExtPhysicalVarWidget(input));
  dialog->addToTab("General", radiusWidget);

  visuWidget = new ExtWidget("OpenMBV Sphere",new OMBVEmptyWidget,true);
  dialog->addToTab("Visualisation", visuWidget);
}

void Sphere::initializeUsingXML(TiXmlElement *element) {
  Contour::initializeUsingXML(element);
  radius.initializeUsingXML(element);
  visu.initializeUsingXML(element);
}

TiXmlElement* Sphere::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e = Contour::writeXMLFile(parent);
  radius.writeXMLFile(e);
  visu.writeXMLFile(e);
  return e;
}

void Sphere::toWidget() {
  Contour::toWidget();
  radius.toWidget(radiusWidget);
  visu.toWidget(visuWidget);
}

void Sphere::fromWidget() {
  Contour::fromWidget();
  radius.fromWidget(radiusWidget);
  visu.fromWidget(visuWidget);
}
