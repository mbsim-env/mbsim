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
#include "function_property_factory.h"
#include "objectfactory.h"
#include "mainwindow.h"
#include "embed.h"

using namespace std;
using namespace MBXMLUtils;
using namespace boost;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  Contour::Contour(const string &str, Element *parent) : Element(str,parent), thickness(0,false) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0.01"), "m", MBSIM%"thickness"));
    thickness.setProperty(new ExtPhysicalVarProperty(input));
  }

  Contour* Contour::readXMLFile(const string &filename, Element *parent) {
    shared_ptr<DOMDocument> doc=mw->parser->parse(filename);
    DOMElement *e=doc->getDocumentElement();
//    Contour *contour=ObjectFactory::getInstance()->createContour(e, parent);
    Contour *contour=Embed<Contour>::createAndInit(e,parent);
    if(contour) {
//      contour->initializeUsingXML(e);
      contour->initialize();
    }
    return contour;
  }

  DOMElement* Contour::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);
    thickness.initializeUsingXML(element);
    return element;
  }

  DOMElement* Contour::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Element::writeXMLFile(parent);
    thickness.writeXMLFile(ele0);
    return ele0;
  }

  RigidContour::RigidContour(const string &str, Element *parent) : Contour(str,parent), refFrame(0,false) {
    refFrame.setProperty(new ParentFrameOfReferenceProperty(getParent()->getFrame(0)->getXMLPath(this,true),this,MBSIM%"frameOfReference"));
  }

  void RigidContour::initialize() {
    Contour::initialize();
    refFrame.initialize();
  }

  void RigidContour::setSavedFrameOfReference(const string &str) {
    ((ParentFrameOfReferenceProperty*)(refFrame.getProperty()))->setFrame(str);
  }

  DOMElement* RigidContour::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    refFrame.initializeUsingXML(element);
    return element;
  }

  DOMElement* RigidContour::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Contour::writeXMLFile(parent);
    refFrame.writeXMLFile(ele0);
    return ele0;
  }

  Point::Point(const string &str, Element *parent) : RigidContour(str,parent) {
    visu.setProperty(new PointMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  DOMElement* Point::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* Point::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    visu.writeXMLFile(e);
    return e;
  }

  Line::Line(const string &str, Element *parent) : RigidContour(str,parent) {
    visu.setProperty(new LineMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  DOMElement* Line::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* Line::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    visu.writeXMLFile(e);
    return e;
  }

  Plane::Plane(const string &str, Element *parent) : RigidContour(str,parent) {

    visu.setProperty(new PlaneMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  DOMElement* Plane::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* Plane::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    visu.writeXMLFile(e);
    return e;
  }

  Sphere::Sphere(const string &str, Element *parent) : RigidContour(str,parent) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIM%"radius"));
    radius.setProperty(new ExtPhysicalVarProperty(input));

    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));

  }

  DOMElement* Sphere::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    radius.initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* Sphere::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    radius.writeXMLFile(e);
    visu.writeXMLFile(e);
    return e;
  }

  Circle::Circle(const string &str, Element *parent) : RigidContour(str,parent), solid(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIM%"radius"));
    radius.setProperty(new ExtPhysicalVarProperty(input));
    solid.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"solid",vector<string>(2,"")),"",4));
    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  DOMElement* Circle::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    radius.initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* Circle::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    radius.writeXMLFile(e);
    visu.writeXMLFile(e);
    return e;
  }

  Cuboid::Cuboid(const string &str, Element *parent) : RigidContour(str,parent) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(3), "m", MBSIM%"length"));
    length.setProperty(new ExtPhysicalVarProperty(input));

    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));

  }

  DOMElement* Cuboid::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    length.initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* Cuboid::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    length.writeXMLFile(e);
    visu.writeXMLFile(e);
    return e;
  }

  LineSegment::LineSegment(const string &str, Element *parent) : RigidContour(str,parent) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIM%"length"));
    length.setProperty(new ExtPhysicalVarProperty(input));

    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));

  }

  DOMElement* LineSegment::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    length.initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* LineSegment::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    length.writeXMLFile(e);
    visu.writeXMLFile(e);
    return e;
  }

  PlanarContour::PlanarContour(const string &str, Element *parent) : RigidContour(str,parent), open(0,false) {

    nodes.setProperty(new ChoiceProperty2(new VecPropertyFactory(2,MBSIM%"nodes",vector<string>(3,"")),"",4));

    contourFunction.setProperty(new ChoiceProperty2(new PlanarContourFunctionPropertyFactory(this),MBSIM%"contourFunction"));

    open.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"open",vector<string>(2,"")),"",4));

    visu.setProperty(new PlanarContourMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  DOMElement* PlanarContour::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    nodes.initializeUsingXML(element);
    contourFunction.initializeUsingXML(element);
    open.initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarContour::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    nodes.writeXMLFile(e);
    contourFunction.writeXMLFile(e);
    open.writeXMLFile(e);
    visu.writeXMLFile(e);
    return e;
  }

  SpatialContour::SpatialContour(const string &str, Element *parent) : RigidContour(str,parent), open(0,false) {

    etaNodes.setProperty(new ChoiceProperty2(new VecPropertyFactory(2,MBSIM%"etaNodes",vector<string>(3,"")),"",4));

    xiNodes.setProperty(new ChoiceProperty2(new VecPropertyFactory(2,MBSIM%"xiNodes",vector<string>(3,"")),"",4));

    contourFunction.setProperty(new ChoiceProperty2(new SpatialContourFunctionPropertyFactory(this),MBSIM%"contourFunction"));

    open.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"open",vector<string>(2,"")),"",4));

    visu.setProperty(new SpatialContourMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  DOMElement* SpatialContour::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    etaNodes.initializeUsingXML(element);
    xiNodes.initializeUsingXML(element);
    contourFunction.initializeUsingXML(element);
    open.initializeUsingXML(element);
    visu.initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialContour::writeXMLFile(DOMNode *parent) {
    DOMElement *e = RigidContour::writeXMLFile(parent);
    etaNodes.writeXMLFile(e);
    xiNodes.writeXMLFile(e);
    contourFunction.writeXMLFile(e);
    open.writeXMLFile(e);
    visu.writeXMLFile(e);
    return e;
  }

}
