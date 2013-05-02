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
#include "variable_properties.h"
#include "function_properties.h"
#include "function_widgets.h"
#include "extended_widgets.h"
#include "utils.h"

using namespace std;
using namespace MBXMLUtils;

TiXmlElement* Function1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  parent->LinkEndChild(ele0);
  return ele0;
}

TiXmlElement* Function2Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  parent->LinkEndChild(ele0);
  return ele0;
}

void DifferentiableFunction1Property::setDerivative(Function1Property *diff,size_t degree) { 
  derivatives.resize(max(derivatives.size(),degree+1)); 
  derivatives[degree]=diff; 
}

ConstantFunction1Property::ConstantFunction1Property(const string &ext) : Function1Property(ext) {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"value"));
  c.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* ConstantFunction1Property::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  return element;
}

TiXmlElement* ConstantFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1Property::writeXMLFile(parent);
  c.writeXMLFile(ele0);
  return ele0;
} 

void ConstantFunction1Property::fromWidget(QWidget *widget) {
  c.fromWidget(static_cast<ConstantFunction1Widget*>(widget)->c);
}

void ConstantFunction1Property::toWidget(QWidget *widget) {
  c.toWidget(static_cast<ConstantFunction1Widget*>(widget)->c);
}

QuadraticFunction1Property::QuadraticFunction1Property() {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"a0"));
  a0.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"a1"));
  a1.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"a2"));
  a2.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* QuadraticFunction1Property::initializeUsingXML(TiXmlElement *element) {
  a0.initializeUsingXML(element);
  a1.initializeUsingXML(element);
  a2.initializeUsingXML(element);
  return element;
}

TiXmlElement* QuadraticFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = DifferentiableFunction1Property::writeXMLFile(parent);
  a0.writeXMLFile(ele0);
  a1.writeXMLFile(ele0);
  a2.writeXMLFile(ele0);
  return ele0;
}

void QuadraticFunction1Property::fromWidget(QWidget *widget) {
  a0.fromWidget(static_cast<QuadraticFunction1Widget*>(widget)->a0);
  a1.fromWidget(static_cast<QuadraticFunction1Widget*>(widget)->a1);
  a2.fromWidget(static_cast<QuadraticFunction1Widget*>(widget)->a2);
}

void QuadraticFunction1Property::toWidget(QWidget *widget) {
  a0.toWidget(static_cast<QuadraticFunction1Widget*>(widget)->a0);
  a1.toWidget(static_cast<QuadraticFunction1Widget*>(widget)->a1);
  a2.toWidget(static_cast<QuadraticFunction1Widget*>(widget)->a2);
}

SinusFunction1Property::SinusFunction1Property() {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"amplitude"));
  a.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"frequency"));
  f.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"phase"));
  p.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"offset"));
  o.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* SinusFunction1Property::initializeUsingXML(TiXmlElement *element) {
  a.initializeUsingXML(element);
  f.initializeUsingXML(element);
  p.initializeUsingXML(element);
  o.initializeUsingXML(element);
  return element;
}

TiXmlElement* SinusFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1Property::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  f.writeXMLFile(ele0);
  p.writeXMLFile(ele0);
  o.writeXMLFile(ele0);
  return ele0;
} 

void SinusFunction1Property::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<SinusFunction1Widget*>(widget)->a);
  f.fromWidget(static_cast<SinusFunction1Widget*>(widget)->f);
  p.fromWidget(static_cast<SinusFunction1Widget*>(widget)->p);
  o.fromWidget(static_cast<SinusFunction1Widget*>(widget)->o);
}

void SinusFunction1Property::toWidget(QWidget *widget) {
  a.toWidget(static_cast<SinusFunction1Widget*>(widget)->a);
  f.toWidget(static_cast<SinusFunction1Widget*>(widget)->f);
  p.toWidget(static_cast<SinusFunction1Widget*>(widget)->p);
  o.toWidget(static_cast<SinusFunction1Widget*>(widget)->o);
}

TabularFunction1Property::TabularFunction1Property() {

  PropertyContainer *propertyContainer = new PropertyContainer;
  vector<Property*> choiceProperty;

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecFromFileProperty,"",MBSIMNS"x"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new MatFromFileProperty,"",MBSIMNS"y"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  choiceProperty.push_back(propertyContainer);

  input.clear();
  input.push_back(new PhysicalVariableProperty(new MatFromFileProperty,"",MBSIMNS"xy"));
  choiceProperty.push_back(new ExtProperty(new ExtPhysicalVarProperty(input)));

  choice = new PropertyChoiceProperty(choiceProperty);
}

TiXmlElement* TabularFunction1Property::initializeUsingXML(TiXmlElement *element) {
  choice->initializeUsingXML(element);
  return element;
}

TiXmlElement* TabularFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1Property::writeXMLFile(parent);
  choice->writeXMLFile(ele0);
  return ele0;
} 

void TabularFunction1Property::fromWidget(QWidget *widget) {
  choice->fromWidget(static_cast<TabularFunction1Widget*>(widget)->choice);
}

void TabularFunction1Property::toWidget(QWidget *widget) {
  choice->toWidget(static_cast<TabularFunction1Widget*>(widget)->choice);
}

TiXmlElement* SummationFunction1Property::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e = element->FirstChildElement(MBSIMNS"function");
  while(e) {
    functionChoice.push_back(new Function1ChoiceProperty("",true));
    functionChoice[functionChoice.size()-1]->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }
  return e;
}

TiXmlElement* SummationFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1Property::writeXMLFile(parent);
  for(int i=0; i<functionChoice.size(); i++) {
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"function");
    ele0->LinkEndChild(ele1);
    functionChoice[i]->writeXMLFile(ele1);
  }
  return ele0;
}

void SummationFunction1Property::fromWidget(QWidget *widget) {
  for(unsigned int i=0; i<static_cast<SummationFunction1Widget*>(widget)->functionChoice.size(); i++) {
    functionChoice.push_back(new Function1ChoiceProperty("",true));
    functionChoice[i]->fromWidget(static_cast<SummationFunction1Widget*>(widget)->functionChoice[i]);
  }
}

void SummationFunction1Property::toWidget(QWidget *widget) {
  for(unsigned int i=0; i<functionChoice.size(); i++) {
    static_cast<SummationFunction1Widget*>(widget)->blockSignals(true);
    static_cast<SummationFunction1Widget*>(widget)->addFunction();
    static_cast<SummationFunction1Widget*>(widget)->blockSignals(false);
    functionChoice[i]->toWidget(static_cast<SummationFunction1Widget*>(widget)->functionChoice[i]);
  }
}

LinearSpringDamperForceProperty::LinearSpringDamperForceProperty() {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIMNS"stiffnessCoefficient"));
  c.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIMNS"dampingCoefficient"));
  d.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"m",MBSIMNS"unloadedLength"));
  l0.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearSpringDamperForceProperty::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  d.initializeUsingXML(element);
  l0.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearSpringDamperForceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function2Property::writeXMLFile(parent);
  c.writeXMLFile(ele0);
  d.writeXMLFile(ele0);
  l0.writeXMLFile(ele0);
  return ele0;
} 

void LinearSpringDamperForceProperty::fromWidget(QWidget *widget) {
  c.fromWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->c);
  d.fromWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->d);
  l0.fromWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->l0);
}

void LinearSpringDamperForceProperty::toWidget(QWidget *widget) {
  c.toWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->c);
  d.toWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->d);
  l0.toWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->l0);
}

LinearRegularizedBilateralConstraintProperty::LinearRegularizedBilateralConstraintProperty() {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIMNS"stiffnessCoefficient"));
  c.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIMNS"dampingCoefficient"));
  d.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearRegularizedBilateralConstraintProperty::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  d.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearRegularizedBilateralConstraintProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function2Property::writeXMLFile(parent);
  c.writeXMLFile(ele0);
  d.writeXMLFile(ele0);
  return ele0;
} 

void LinearRegularizedBilateralConstraintProperty::fromWidget(QWidget *widget) {
  c.fromWidget(static_cast<LinearRegularizedBilateralConstraintWidget*>(widget)->c);
  d.fromWidget(static_cast<LinearRegularizedBilateralConstraintWidget*>(widget)->d);
}

void LinearRegularizedBilateralConstraintProperty::toWidget(QWidget *widget) {
  c.toWidget(static_cast<LinearRegularizedBilateralConstraintWidget*>(widget)->c);
  d.toWidget(static_cast<LinearRegularizedBilateralConstraintWidget*>(widget)->d);
}

LinearRegularizedUnilateralConstraintProperty::LinearRegularizedUnilateralConstraintProperty() {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIMNS"stiffnessCoefficient"));
  c.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIMNS"dampingCoefficient"));
  d.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearRegularizedUnilateralConstraintProperty::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  d.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearRegularizedUnilateralConstraintProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function2Property::writeXMLFile(parent);
  c.writeXMLFile(ele0);
  d.writeXMLFile(ele0);
  return ele0;
} 

void LinearRegularizedUnilateralConstraintProperty::fromWidget(QWidget *widget) {
  c.fromWidget(static_cast<LinearRegularizedUnilateralConstraintWidget*>(widget)->c);
  d.fromWidget(static_cast<LinearRegularizedUnilateralConstraintWidget*>(widget)->d);
}

void LinearRegularizedUnilateralConstraintProperty::toWidget(QWidget *widget) {
  c.toWidget(static_cast<LinearRegularizedUnilateralConstraintWidget*>(widget)->c);
  d.toWidget(static_cast<LinearRegularizedUnilateralConstraintWidget*>(widget)->d);
}

LinearRegularizedCoulombFrictionProperty::LinearRegularizedCoulombFrictionProperty() : gd(0,false) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0.01"),"m/s",MBSIMNS"marginalVelocity"));
  gd.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"frictionCoefficient"));
  mu.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearRegularizedCoulombFrictionProperty::initializeUsingXML(TiXmlElement *element) {
  gd.initializeUsingXML(element);
  mu.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearRegularizedCoulombFrictionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function2Property::writeXMLFile(parent);
  gd.writeXMLFile(ele0);
  mu.writeXMLFile(ele0);
  return ele0;
} 

void LinearRegularizedCoulombFrictionProperty::fromWidget(QWidget *widget) {
  gd.fromWidget(static_cast<LinearRegularizedCoulombFrictionWidget*>(widget)->gd);
  mu.fromWidget(static_cast<LinearRegularizedCoulombFrictionWidget*>(widget)->mu);
}

void LinearRegularizedCoulombFrictionProperty::toWidget(QWidget *widget) {
  gd.toWidget(static_cast<LinearRegularizedCoulombFrictionWidget*>(widget)->gd);
  mu.toWidget(static_cast<LinearRegularizedCoulombFrictionWidget*>(widget)->mu);
}

Function1ChoiceProperty::Function1ChoiceProperty(const string &xmlName_, bool withFactor) : function(0), factor(0), index(0), xmlName(xmlName_) {

  if(withFactor) {
    vector<PhysicalVariableProperty*> input;
    input.push_back(new PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMNS"factor"));
    factor.setProperty(new ExtPhysicalVarProperty(input));
  }
  defineForceLaw(0);
}

void Function1ChoiceProperty::defineForceLaw(int index_) {
  index = index_;
  delete function;
  if(index==0)
    function = new ConstantFunction1Property("VS");  
  else if(index==1)
    function = new QuadraticFunction1Property;
  else if(index==2)
    function = new SinusFunction1Property;
  else if(index==3)
    function = new TabularFunction1Property;
  else if(index==4) {
    function = new SummationFunction1Property;
  }
}

TiXmlElement* Function1ChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=xmlName!=""?element->FirstChildElement(xmlName):element;
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"ConstantFunction1_VS")
        index = 0;
      else if(ee->ValueStr() == MBSIMNS"QuadraticFunction1_VS")
        index = 1;
      else if(ee->ValueStr() == MBSIMNS"SinusFunction1_VS")
        index = 2;
      else if(ee->ValueStr() == MBSIMNS"TabularFunction1_VS")
        index = 3;
      else if(ee->ValueStr() == MBSIMNS"SummationFunction1_VS")
        index = 4;
      defineForceLaw(index);
      function->initializeUsingXML(ee);
    }
    if(factor.getProperty())
      factor.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* Function1ChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlNode *ele0;
  if(xmlName!="") {
    ele0 = new TiXmlElement(xmlName);
    parent->LinkEndChild(ele0);
  }
  else
    ele0 = parent;
  if(function)
    function->writeXMLFile(ele0);
  if(factor.getProperty())
    factor.writeXMLFile(ele0);

  return 0;
}

void Function1ChoiceProperty::fromWidget(QWidget *widget) {
  defineForceLaw(static_cast<Function1ChoiceWidget*>(widget)->comboBox->currentIndex());
  function->fromWidget(static_cast<Function1ChoiceWidget*>(widget)->function);
  if(factor.getProperty())
    factor.fromWidget(static_cast<Function1ChoiceWidget*>(widget)->factor);
}

void Function1ChoiceProperty::toWidget(QWidget *widget) {
  static_cast<Function1ChoiceWidget*>(widget)->comboBox->blockSignals(true);
  static_cast<Function1ChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<Function1ChoiceWidget*>(widget)->comboBox->blockSignals(false);
  static_cast<Function1ChoiceWidget*>(widget)->blockSignals(true);
  static_cast<Function1ChoiceWidget*>(widget)->defineForceLaw(index);
  static_cast<Function1ChoiceWidget*>(widget)->blockSignals(false);
  function->toWidget(static_cast<Function1ChoiceWidget*>(widget)->function);
  if(factor.getProperty())
    factor.toWidget(static_cast<Function1ChoiceWidget*>(widget)->factor);
}

Function2ChoiceProperty::Function2ChoiceProperty(const string &xmlName_) : function(0), index(0), xmlName(xmlName_) {
  defineForceLaw(0);
}

void Function2ChoiceProperty::defineForceLaw(int index_) {
  index = index_;
  delete function;
  if(index==0)
    function = new LinearSpringDamperForceProperty;  
}

TiXmlElement* Function2ChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"LinearSpringDamperForce") {
        defineForceLaw(0);
        function->initializeUsingXML(ee);
      }
    }
  }
  return e;
}

TiXmlElement* Function2ChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  if(function)
    function->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

void Function2ChoiceProperty::fromWidget(QWidget *widget) {
  defineForceLaw(static_cast<Function2ChoiceWidget*>(widget)->comboBox->currentIndex());
  function->fromWidget(static_cast<Function2ChoiceWidget*>(widget)->function);
}

void Function2ChoiceProperty::toWidget(QWidget *widget) {
  static_cast<Function2ChoiceWidget*>(widget)->comboBox->blockSignals(true);
  static_cast<Function2ChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<Function2ChoiceWidget*>(widget)->comboBox->blockSignals(false);
  static_cast<Function2ChoiceWidget*>(widget)->blockSignals(true);
  static_cast<Function2ChoiceWidget*>(widget)->defineForceLaw(index);
  static_cast<Function2ChoiceWidget*>(widget)->blockSignals(false);
  function->toWidget(static_cast<Function2ChoiceWidget*>(widget)->function);
}
