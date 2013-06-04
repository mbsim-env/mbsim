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
#include "frame.h"
#include "basic_properties.h"
#include "ombv_properties.h"
#include "objectfactory.h"

using namespace std;
using namespace MBXMLUtils;

Contour::Contour(const string &str, Element *parent) : Element(str,parent), refFrame(0,false) {
  refFrame.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIMNS"frameOfReference"));
}

Contour::~Contour() {
}

void Contour::initialize() {
  Element::initialize();
  refFrame.initialize();
}

void Contour::setSavedFrameOfReference(const string &str) {
  ((ParentFrameOfReferenceProperty*)(refFrame.getProperty()))->setFrame(str);
}

Contour* Contour::readXMLFile(const string &filename, Element *parent) {
  TiXmlDocument doc;
  if(doc.LoadFile(filename)) {
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    Contour *contour=ObjectFactory::getInstance()->createContour(e,parent);
    if(contour)
      contour->initializeUsingXML(e);
    return contour;
  }
  return 0;
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
  return NULL;
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

  visu.setProperty(new OMBVPlaneProperty(MBSIMNS"enableOpenMBV",getID()));
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
 
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIMNS"radius"));
  radius.setProperty(new ExtPhysicalVarProperty(input));

  visu.setProperty(new OMBVEmptyProperty(MBSIMNS"enableOpenMBV",getID()));

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

CircleSolid::CircleSolid(const string &str, Element *parent) : Contour(str,parent) {
 
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIMNS"radius"));
  radius.setProperty(new ExtPhysicalVarProperty(input));

  visu.setProperty(new OMBVEmptyProperty(MBSIMNS"enableOpenMBV",getID()));

}

CircleSolid::~CircleSolid() {
}

void CircleSolid::initializeUsingXML(TiXmlElement *element) {
  Contour::initializeUsingXML(element);
  radius.initializeUsingXML(element);
  visu.initializeUsingXML(element);
}

TiXmlElement* CircleSolid::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *e = Contour::writeXMLFile(parent);
  radius.writeXMLFile(e);
  visu.writeXMLFile(e);
  return e;
}
