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
#include "mainwindow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace boost;
using namespace xercesc;

namespace MBSimGUI {

  Contour::Contour(const string &str, Element *parent) : Element(str,parent), refFrame(0,false) {
    refFrame.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIM%"frameOfReference"));
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
    shared_ptr<DOMDocument> doc=MainWindow::parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
    Contour *contour=ObjectFactory::getInstance()->createContour(e, parent);
    if(contour) {
      contour->initializeUsingXML(e);
      contour->initialize();
    }
    return contour;
  }

  void Contour::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);
    refFrame.initializeUsingXML(element);
  }

  DOMElement* Contour::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Element::writeXMLFile(parent);
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
    visu.setProperty(new PointMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  Point::~Point() {
  }

  void Point::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    visu.initializeUsingXML(element);
  }

  DOMElement* Point::writeXMLFile(DOMNode *parent) {
    DOMElement *e = Contour::writeXMLFile(parent);
    visu.writeXMLFile(e);
    return e;
  }

  Line::Line(const string &str, Element *parent) : Contour(str,parent) {
    visu.setProperty(new LineMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  Line::~Line() {
  }

  void Line::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    visu.initializeUsingXML(element);
  }

  DOMElement* Line::writeXMLFile(DOMNode *parent) {
    DOMElement *e = Contour::writeXMLFile(parent);
    visu.writeXMLFile(e);
    return e;
  }

  Plane::Plane(const string &str, Element *parent) : Contour(str,parent) {

    visu.setProperty(new PlaneMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  Plane::~Plane() {
  }

  void Plane::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    visu.initializeUsingXML(element);
  }

  DOMElement* Plane::writeXMLFile(DOMNode *parent) {
    DOMElement *e = Contour::writeXMLFile(parent);
    visu.writeXMLFile(e);
    return e;
  }

  Sphere::Sphere(const string &str, Element *parent) : Contour(str,parent) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIM%"radius"));
    radius.setProperty(new ExtPhysicalVarProperty(input));

    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));

  }

  Sphere::~Sphere() {
  }

  void Sphere::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    radius.initializeUsingXML(element);
    visu.initializeUsingXML(element);
  }

  DOMElement* Sphere::writeXMLFile(DOMNode *parent) {
    DOMElement *e = Contour::writeXMLFile(parent);
    radius.writeXMLFile(e);
    visu.writeXMLFile(e);
    return e;
  }

  CircleSolid::CircleSolid(const string &str, Element *parent) : Contour(str,parent) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIM%"radius"));
    radius.setProperty(new ExtPhysicalVarProperty(input));

    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));

  }

  CircleSolid::~CircleSolid() {
  }

  void CircleSolid::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    radius.initializeUsingXML(element);
    visu.initializeUsingXML(element);
  }

  DOMElement* CircleSolid::writeXMLFile(DOMNode *parent) {
    DOMElement *e = Contour::writeXMLFile(parent);
    radius.writeXMLFile(e);
    visu.writeXMLFile(e);
    return e;
  }

}
