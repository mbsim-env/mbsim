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
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  Contour::Contour(const QString &str) : Element(str) {
  }

  DOMElement* Contour::processFileID(DOMElement *element) {
    DOMElement *ELE=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ELE) {
      DOMDocument *doc=element->getOwnerDocument();
      DOMProcessingInstruction *id=doc->createProcessingInstruction(X()%"OPENMBV_ID", X()%getID().toStdString());
      ELE->insertBefore(id, NULL);
    }
    return element;
  }

  RigidContour::RigidContour(const QString &str) : Contour(str) {
  }

  Point::Point(const QString &str) : RigidContour(str) {
//    visu.setProperty(new PointMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  Line::Line(const QString &str) : RigidContour(str) {
//    visu.setProperty(new LineMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  Plane::Plane(const QString &str) : RigidContour(str) {

  //  visu.setProperty(new PlaneMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  Sphere::Sphere(const QString &str) : RigidContour(str) {

//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIM%"radius"));
//    radius.setProperty(new ExtPhysicalVarProperty(input));
//
//    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));

  }

  Circle::Circle(const QString &str) : RigidContour(str) {

//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIM%"radius"));
//    radius.setProperty(new ExtPhysicalVarProperty(input));
//    solid.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"solid",vector<string>(2,"")),"",4));
//    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  Cuboid::Cuboid(const QString &str) : RigidContour(str) {

//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new VecProperty(3), "m", MBSIM%"length"));
//    length.setProperty(new ExtPhysicalVarProperty(input));
//
//    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));

  }

  LineSegment::LineSegment(const QString &str) : RigidContour(str) {

//    vector<PhysicalVariableProperty> input;
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"), "m", MBSIM%"length"));
//    length.setProperty(new ExtPhysicalVarProperty(input));
//
//    visu.setProperty(new MBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));

  }

  PlanarContour::PlanarContour(const QString &str) : RigidContour(str) {
//
//    nodes.setProperty(new ChoiceProperty2(new VecPropertyFactory(2,MBSIM%"nodes",vector<string>(3,"")),"",4));
//
//    contourFunction.setProperty(new ChoiceProperty2(new PlanarContourFunctionPropertyFactory(this),MBSIM%"contourFunction"));
//
//    open.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"open",vector<string>(2,"")),"",4));
//
//    visu.setProperty(new PlanarContourMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

  SpatialContour::SpatialContour(const QString &str) : RigidContour(str) {

//    etaNodes.setProperty(new ChoiceProperty2(new VecPropertyFactory(2,MBSIM%"etaNodes",vector<string>(3,"")),"",4));
//
//    xiNodes.setProperty(new ChoiceProperty2(new VecPropertyFactory(2,MBSIM%"xiNodes",vector<string>(3,"")),"",4));
//
//    contourFunction.setProperty(new ChoiceProperty2(new SpatialContourFunctionPropertyFactory(this),MBSIM%"contourFunction"));
//
//    open.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"open",vector<string>(2,"")),"",4));
//
//    visu.setProperty(new SpatialContourMBSOMBVProperty("NOTSET",MBSIM%"enableOpenMBV",getID()));
  }

}
