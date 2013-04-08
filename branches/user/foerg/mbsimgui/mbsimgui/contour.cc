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

Contour::Contour(const string &str, Element *parent) : Element(str,parent), refFrame(0,false) {
  refFrame.setProperty(new ParentFrameOfReferenceProperty(0,this,MBSIMNS"frameOfReference"));
}

Contour::~Contour() {
}

void Contour::initialize() {
  Element::initialize();
  refFrame.initialize();
}

void Contour::setSavedFrameOfReference(const string &str) {
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

Element *Contour::getByPathSearch(string path) {
  if (path.substr(0, 1)=="/") // absolut path
    if(getParent())
      return getParent()->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  else if (path.substr(0, 3)=="../") // relative path
    return getParent()->getByPathSearch(path.substr(3));
  else { // local path
    throw;
  }
}

Point::Point(const string &str, Element *parent) : Contour(str,parent) {
}

Point::~Point() {
}

Line::Line(const string &str, Element *parent) : Contour(str,parent) {
}

Line::~Line() {
}

Plane::Plane(const string &str, Element *parent) : Contour(str,parent) {

  visu.setProperty(new OMBVPlaneProperty(MBSIMNS"enableOpenMBV"));
  ((OMBVPlaneProperty*)visu.getProperty())->setID(getID());
}

Plane::~Plane() {
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

Sphere::Sphere(const string &str, Element *parent) : Contour(str,parent) {
 
  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1"), "m", MBSIMNS"radius"));
  radius.setProperty(new ExtPhysicalVarProperty(input));

  visu.setProperty(new OMBVEmptyProperty(MBSIMNS"enableOpenMBV"));
  ((OMBVEmptyProperty*)visu.getProperty())->setID(getID());

}

Sphere::~Sphere() {
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
