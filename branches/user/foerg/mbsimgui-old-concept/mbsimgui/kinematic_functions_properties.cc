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

TranslationAlongFixedAxisProperty::TranslationAlongFixedAxisProperty() {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecProperty(3),"",MBSIM%"axisOfTranslation"));
  a.setProperty(new ExtPhysicalVarProperty(input));
}

DOMElement* TranslationAlongFixedAxisProperty::initializeUsingXML(DOMElement *element) {
  a.initializeUsingXML(element);
  return element;
}

DOMElement* TranslationAlongFixedAxisProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  return ele0;
} 

void TranslationAlongFixedAxisProperty::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<TranslationAlongFixedAxisWidget*>(widget)->a);
}

void TranslationAlongFixedAxisProperty::toWidget(QWidget *widget) {
  a.toWidget(static_cast<TranslationAlongFixedAxisWidget*>(widget)->a);
}

LinearTranslationProperty::LinearTranslationProperty(int m, int n) : b(0,false) {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new MatProperty(m,n),"",MBSIM%"translationVectors"));
  A.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIM%"offset"));
  b.setProperty(new ExtPhysicalVarProperty(input));
}

int LinearTranslationProperty::getArg1Size() const {
  string str = OctEval::cast<string>(MainWindow::octEval->stringToOctValue(static_cast<const ExtPhysicalVarProperty*>(A.getProperty())->getCurrentPhysicalVariableProperty().getValue()));
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

DOMElement* LinearTranslationProperty::initializeUsingXML(DOMElement *element) {
  A.initializeUsingXML(element);
  b.initializeUsingXML(element);
  return element;
}

DOMElement* LinearTranslationProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele0 = FunctionProperty::writeXMLFile(parent);
  A.writeXMLFile(ele0);
  b.writeXMLFile(ele0);
  return ele0;
} 

void LinearTranslationProperty::fromWidget(QWidget *widget) {
  A.fromWidget(static_cast<LinearTranslationWidget*>(widget)->A);
  b.fromWidget(static_cast<LinearTranslationWidget*>(widget)->b);
}

void LinearTranslationProperty::toWidget(QWidget *widget) {
  A.toWidget(static_cast<LinearTranslationWidget*>(widget)->A);
  b.toWidget(static_cast<LinearTranslationWidget*>(widget)->b);
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


