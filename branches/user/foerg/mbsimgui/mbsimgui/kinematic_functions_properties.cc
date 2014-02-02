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
#include "kinematic_functions_properties.h"
#include "kinematic_functions_widgets.h"
#include "extended_widgets.h"
#include "mainwindow.h"
#include <mbxmlutils/octeval.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

TranslationAlongFixedAxisProperty::TranslationAlongFixedAxisProperty(const string &name) : FunctionProperty(name) {
  vector<string> x = getVec<string>(3,"0");
  x[0] = "1";
  property.push_back(new Vec_Property("axisOfTranslation",x));
}

DOMElement* TranslationAlongFixedAxisProperty::initializeUsingXML(DOMElement *element) {
  DOMElement *ele1 = E(element)->getFirstElementChildNamed( MBSIM%"axisOfTranslation" );
  property[0]->initializeUsingXML(ele1);
  return element;
}

DOMElement* TranslationAlongFixedAxisProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0 = FunctionProperty::writeXMLFile(parent);
  DOMElement *ele1 = D(doc)->createElement( MBSIM%"axisOfTranslation" );
  property[0]->writeXMLFile(ele1);
  ele0->insertBefore(ele1, NULL);
  return ele0;
} 

void TranslationAlongFixedAxisProperty::fromWidget(QWidget *widget) {
}

void TranslationAlongFixedAxisProperty::toWidget(QWidget *widget) {
}

LinearTranslationProperty::LinearTranslationProperty(const string &name) : FunctionProperty(name) {
  vector<vector<string> > A = getMat<string>(3,1,"0");
  A[0][0] = "1";
  addProperty(new VarMat_Property("translationVectors",A,Units()));
  addProperty(new Vec_Property("offset",Units()));
}

int LinearTranslationProperty::getArgSize(int i) const {
  return static_cast<VarMat_Property*>(property[0])->size(1);
}

DOMElement* LinearTranslationProperty::initializeUsingXML(DOMElement *element) {
}

DOMElement* LinearTranslationProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0 = FunctionProperty::writeXMLFile(parent);
  DOMElement *ele1 = D(doc)->createElement( MBSIM%"translationVectors" );
  property[0]->writeXMLFile(ele1);
  ele0->insertBefore(ele1, NULL);
  ele1 = D(doc)->createElement( MBSIM%"offset" );
  property[1]->writeXMLFile(ele1);
  ele0->insertBefore(ele1, NULL);
  return ele0;
} 

void LinearTranslationProperty::fromWidget(QWidget *widget) {
}

void LinearTranslationProperty::toWidget(QWidget *widget) {
}

RotationAboutFixedAxisProperty::RotationAboutFixedAxisProperty() {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecProperty(3),"",MBSIM%"axisOfRotation"));
  a.setProperty(new ExtPhysicalVarProperty(input));
}

DOMElement* RotationAboutFixedAxisProperty::initializeUsingXML(DOMElement *element) {
  a.initializeUsingXML(element);
  return element;
}

DOMElement* RotationAboutFixedAxisProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  return ele0;
} 

void RotationAboutFixedAxisProperty::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<RotationAboutFixedAxisWidget*>(widget)->a);
}

void RotationAboutFixedAxisProperty::toWidget(QWidget *widget) {
  a.toWidget(static_cast<RotationAboutFixedAxisWidget*>(widget)->a);
}


