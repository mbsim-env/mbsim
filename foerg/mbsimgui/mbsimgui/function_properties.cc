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
#include "basic_properties.h"
#include "variable_properties.h"
#include "function_properties.h"
#include "function_widgets.h"
#include "basic_widgets.h"
#include "extended_widgets.h"
#include "utils.h"
#include "octaveutils.h"
#include <QSpinBox>
#include <QStackedWidget>

using namespace std;
using namespace MBXMLUtils;

TiXmlElement* FunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  parent->LinkEndChild(ele0);
  return ele0;
}

//TiXmlElement* Function2Property::writeXMLFile(TiXmlNode *parent) {
//  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
//  parent->LinkEndChild(ele0);
//  return ele0;
//}

SymbolicFunction1Property::SymbolicFunction1Property(const string &ext, const string &var) : FunctionProperty(ext), argname(1), argdim(ext.size()-1) {
  argname[0].setProperty(new TextProperty(var,""));
  argdim[0].setProperty(new IntegerProperty(1,""));
  f.setProperty(new OctaveExpressionProperty);
}

int SymbolicFunction1Property::getArg1Size() const {
  return static_cast<const IntegerProperty*>(argdim[0].getProperty())->getValue();
}

TiXmlElement* SymbolicFunction1Property::initializeUsingXML(TiXmlElement *element) {
  f.initializeUsingXML(element);
  string str = "arg1name";
  if(element->Attribute(str))
    static_cast<TextProperty*>(argname[0].getProperty())->setText(element->Attribute(str.c_str()));
  str = "arg1dim";
  if(element->Attribute(str))
    static_cast<IntegerProperty*>(argdim[0].getProperty())->setValue(atoi(element->Attribute(str.c_str())));
  return element;
}

TiXmlElement* SymbolicFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  ele0->SetAttribute("arg1name", static_cast<TextProperty*>(argname[0].getProperty())->getText());
  if(ext[1]=='V')
    ele0->SetAttribute("arg1dim",static_cast<IntegerProperty*>(argdim[0].getProperty())->getValue());
  f.writeXMLFile(ele0);
  return ele0;
} 

void SymbolicFunction1Property::fromWidget(QWidget *widget) {
  argname[0].fromWidget(static_cast<SymbolicFunction1Widget*>(widget)->argname[0]);
  argdim[0].fromWidget(static_cast<SymbolicFunction1Widget*>(widget)->argdim[0]);
  f.fromWidget(static_cast<SymbolicFunction1Widget*>(widget)->f);
}

void SymbolicFunction1Property::toWidget(QWidget *widget) {
  argname[0].toWidget(static_cast<SymbolicFunction1Widget*>(widget)->argname[0]);
  argdim[0].toWidget(static_cast<SymbolicFunction1Widget*>(widget)->argdim[0]);
  f.toWidget(static_cast<SymbolicFunction1Widget*>(widget)->f);
}

int SymbolicFunction1Property::getArgDim() const {
  return static_cast<const IntegerProperty*>(argdim[0].getProperty())->getValue();
}

ConstantFunction1Property::ConstantFunction1Property(const string &ext, int m) : FunctionProperty(ext) {
  vector<PhysicalVariableProperty> input;
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"value"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"value"));
  c.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* ConstantFunction1Property::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  return element;
}

TiXmlElement* ConstantFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  c.writeXMLFile(ele0);
  return ele0;
} 

void ConstantFunction1Property::fromWidget(QWidget *widget) {
  c.fromWidget(static_cast<ConstantFunction1Widget*>(widget)->c);
}

void ConstantFunction1Property::toWidget(QWidget *widget) {
  c.toWidget(static_cast<ConstantFunction1Widget*>(widget)->c);
}

LinearFunctionTestProperty::LinearFunctionTestProperty(const string &ext, int m, int n) : FunctionProperty(ext), b(0,false) {

  vector<Property*> property;

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new MatProperty(m,n),"",MBSIMNS"slope"));
  property.push_back(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"slope"));
  property.push_back(new ExtPhysicalVarProperty(input));

  choice.setProperty(new ChoiceProperty("",property));

  input.clear();
//  if(ext[0]=='V' and ext[1]=='V')
    input.push_back(PhysicalVariableProperty(new MatProperty(m,n),"",MBSIMNS"slope"));
//  else
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"value"));
  a.setProperty(new ExtPhysicalVarProperty(input));
//  if(ext[0]=='V' and ext[1]=='V')
   input.clear();
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"intercept"));
//  else
//    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"value"));
  b.setProperty(new ExtPhysicalVarProperty(input));
}

int LinearFunction1Property::getArg1Size() const {
  if(ext[0]=='V' and ext[1]=='V') {
    string str = evalOctaveExpression(static_cast<const ExtPhysicalVarProperty*>(a.getProperty())->getCurrentPhysicalVariableProperty().getValue());
    vector<vector<string> > A = strToMat(str);
    return A.size()?A[0].size():0;
  }
  return 0;
}

TiXmlElement* LinearFunctionTestProperty::initializeUsingXML(TiXmlElement *element) {
  choice.initializeUsingXML(element);
  a.initializeUsingXML(element);
  b.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearFunctionTestProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  choice.writeXMLFile(ele0);
  a.writeXMLFile(ele0);
  b.writeXMLFile(ele0);
  return ele0;
} 

void LinearFunctionTestProperty::fromWidget(QWidget *widget) {
  choice.fromWidget(static_cast<LinearFunctionTestWidget*>(widget)->choice);
  a.fromWidget(static_cast<LinearFunctionTestWidget*>(widget)->a);
  b.fromWidget(static_cast<LinearFunctionTestWidget*>(widget)->b);
}

void LinearFunctionTestProperty::toWidget(QWidget *widget) {
  choice.toWidget(static_cast<LinearFunctionTestWidget*>(widget)->choice);
  a.toWidget(static_cast<LinearFunctionTestWidget*>(widget)->a);
  b.toWidget(static_cast<LinearFunctionTestWidget*>(widget)->b);
}

LinearFunction1Property::LinearFunction1Property(const string &ext, int m, int n) : FunctionProperty(ext), b(0,false) {
  vector<PhysicalVariableProperty> input;
  if(ext[0]=='V' and ext[1]=='V')
    input.push_back(PhysicalVariableProperty(new MatProperty(m,n),"",MBSIMNS"slope"));
  else if(ext[0]=='V' and ext[1]=='S')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"slope"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"",MBSIMNS"slope"));
  a.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"intercept"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"intercept"));
  b.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearFunction1Property::initializeUsingXML(TiXmlElement *element) {
  a.initializeUsingXML(element);
  b.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  b.writeXMLFile(ele0);
  return ele0;
} 

void LinearFunction1Property::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<LinearFunction1Widget*>(widget)->a);
  b.fromWidget(static_cast<LinearFunction1Widget*>(widget)->b);
}

void LinearFunction1Property::toWidget(QWidget *widget) {
  a.toWidget(static_cast<LinearFunction1Widget*>(widget)->a);
  b.toWidget(static_cast<LinearFunction1Widget*>(widget)->b);
}

RotationAboutFixedAxisProperty::RotationAboutFixedAxisProperty(const string &ext) : FunctionProperty(ext) {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecProperty(3),"",MBSIMNS"axisOfRotation"));
  a.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* RotationAboutFixedAxisProperty::initializeUsingXML(TiXmlElement *element) {
  a.initializeUsingXML(element);
  return element;
}

TiXmlElement* RotationAboutFixedAxisProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  return ele0;
} 

void RotationAboutFixedAxisProperty::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<RotationAboutFixedAxisWidget*>(widget)->a);
}

void RotationAboutFixedAxisProperty::toWidget(QWidget *widget) {
  a.toWidget(static_cast<RotationAboutFixedAxisWidget*>(widget)->a);
}

QuadraticFunction1Property::QuadraticFunction1Property() {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"a0"));
  a0.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"a1"));
  a1.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"a2"));
  a2.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* QuadraticFunction1Property::initializeUsingXML(TiXmlElement *element) {
  a0.initializeUsingXML(element);
  a1.initializeUsingXML(element);
  a2.initializeUsingXML(element);
  return element;
}

TiXmlElement* QuadraticFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
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

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"amplitude"));
  a.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"frequency"));
  f.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"phase"));
  p.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(1),"",MBSIMNS"offset"));
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
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
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

TabularFunction1Property::TabularFunction1Property() : choice("",vector<Property*>(),2) {

  ContainerProperty *propertyContainer = new ContainerProperty;
  vector<Property*> choiceProperty;

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new VecFromFileProperty,"",MBSIMNS"x"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  input.clear();
  input.push_back(PhysicalVariableProperty(new MatFromFileProperty,"",MBSIMNS"y"));
  propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

  choiceProperty.push_back(propertyContainer);

  input.clear();
  input.push_back(PhysicalVariableProperty(new MatFromFileProperty,"",MBSIMNS"xy"));
  choiceProperty.push_back(new ExtProperty(new ExtPhysicalVarProperty(input)));

  choice.setProperty(choiceProperty);
}

TiXmlElement* TabularFunction1Property::initializeUsingXML(TiXmlElement *element) {
  choice.initializeUsingXML(element);
  return element;
}

TiXmlElement* TabularFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  choice.writeXMLFile(ele0);
  return ele0;
} 

void TabularFunction1Property::fromWidget(QWidget *widget) {
  choice.fromWidget(static_cast<TabularFunction1Widget*>(widget)->choice);
}

void TabularFunction1Property::toWidget(QWidget *widget) {
  choice.toWidget(static_cast<TabularFunction1Widget*>(widget)->choice);
}

TiXmlElement* SummationFunction1Property::initializeUsingXML(TiXmlElement *element) {
  
  function.clear();
  TiXmlElement *e = element->FirstChildElement(MBSIMNS"function");
  while(e) {
    function.push_back(ContainerProperty());
    vector<Property*> property;
    property.push_back(new ConstantFunction1Property("VS"));
    property.push_back(new QuadraticFunction1Property);
    property.push_back(new SinusFunction1Property);
    property.push_back(new TabularFunction1Property);
    property.push_back(new SummationFunction1Property);
    property.push_back(new SymbolicFunction1Property("VS","t"));
    function[function.size()-1].addProperty(new ChoiceProperty("",property));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMNS"factor"));
    function[function.size()-1].addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

    function[function.size()-1].initializeUsingXML(e);

    e=e->NextSiblingElement();
  }

  return e;
}

TiXmlElement* SummationFunction1Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  for(int i=0; i<function.size(); i++) {
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"function");
    function[i].writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  return ele0;
}

void SummationFunction1Property::fromWidget(QWidget *widget) {
  function.clear();
  for(unsigned int i=0; i<static_cast<SummationFunction1Widget*>(widget)->stackedWidget->count(); i++) {
    function.push_back(ContainerProperty());

    vector<Property*> property;
    property.push_back(new ConstantFunction1Property("VS"));
    property.push_back(new QuadraticFunction1Property);
    property.push_back(new SinusFunction1Property);
    property.push_back(new TabularFunction1Property);
    property.push_back(new SummationFunction1Property);
    property.push_back(new SymbolicFunction1Property("VS","t"));
    function[function.size()-1].addProperty(new ChoiceProperty("",property));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMNS"factor"));
    function[function.size()-1].addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

    function[i].fromWidget(static_cast<SummationFunction1Widget*>(widget)->stackedWidget->widget(i));
  }
}

void SummationFunction1Property::toWidget(QWidget *widget) {
  for(unsigned int i=0; i<function.size(); i++) {
    static_cast<SummationFunction1Widget*>(widget)->blockSignals(true);
    static_cast<SummationFunction1Widget*>(widget)->addFunction();
    static_cast<SummationFunction1Widget*>(widget)->blockSignals(false);
    function[i].toWidget(static_cast<SummationFunction1Widget*>(widget)->stackedWidget->widget(i));
  }
}

SymbolicFunction2Property::SymbolicFunction2Property(const string &ext, const vector<string> &var) : FunctionProperty(ext), argname(ext.size()-1), argdim(ext.size()-1) {
  for(int i=1; i<ext.size(); i++) {
     argname[i-1].setProperty(new TextProperty(var[i-1],""));
     argdim[i-1].setProperty(new IntegerProperty(1,""));
  }
  f.setProperty(new OctaveExpressionProperty);
}

TiXmlElement* SymbolicFunction2Property::initializeUsingXML(TiXmlElement *element) {
  f.initializeUsingXML(element);
  for(int i=1; i<ext.size(); i++) {
    string str = "arg"+toStr(i)+"name";
    if(element->Attribute(str))
      static_cast<TextProperty*>(argname[i-1].getProperty())->setText(element->Attribute(str.c_str()));
    str = "arg"+toStr(i)+"dim";
    if(element->Attribute(str))
      static_cast<IntegerProperty*>(argdim[i-1].getProperty())->setValue(atoi(element->Attribute(str.c_str())));
  }
  return element;
}

TiXmlElement* SymbolicFunction2Property::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  for(int i=1; i<ext.size(); i++) {
    string istr = ext.size()>2?toStr(i):"";
    ele0->SetAttribute("arg"+istr+"name", static_cast<TextProperty*>(argname[i-1].getProperty())->getText());
    if(ext[i]=='V')
      ele0->SetAttribute("arg"+istr+"dim",static_cast<IntegerProperty*>(argdim[i-1].getProperty())->getValue());
  }
  f.writeXMLFile(ele0);
  return ele0;
} 

void SymbolicFunction2Property::fromWidget(QWidget *widget) {
  for(int i=0; i<argname.size(); i++) {
    argname[i].fromWidget(static_cast<SymbolicFunction2Widget*>(widget)->argname[i]);
    argdim[i].fromWidget(static_cast<SymbolicFunction2Widget*>(widget)->argdim[i]);
  }
  f.fromWidget(static_cast<SymbolicFunction2Widget*>(widget)->f);
}

void SymbolicFunction2Property::toWidget(QWidget *widget) {
  for(int i=0; i<argname.size(); i++) {
    argname[i].toWidget(static_cast<SymbolicFunction2Widget*>(widget)->argname[i]);
    argdim[i].toWidget(static_cast<SymbolicFunction2Widget*>(widget)->argdim[i]);
  }
  f.toWidget(static_cast<SymbolicFunction2Widget*>(widget)->f);
}

int SymbolicFunction2Property::getArg1Size() const {
  return static_cast<const IntegerProperty*>(argdim[0].getProperty())->getValue();
}

int SymbolicFunction2Property::getArg2Size() const {
  return static_cast<const IntegerProperty*>(argdim[1].getProperty())->getValue();
}

LinearSpringDamperForceProperty::LinearSpringDamperForceProperty() {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIMNS"stiffnessCoefficient"));
  c.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIMNS"dampingCoefficient"));
  d.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"m",MBSIMNS"unloadedLength"));
  l0.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearSpringDamperForceProperty::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  d.initializeUsingXML(element);
  l0.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearSpringDamperForceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
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

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIMNS"stiffnessCoefficient"));
  c.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIMNS"dampingCoefficient"));
  d.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearRegularizedBilateralConstraintProperty::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  d.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearRegularizedBilateralConstraintProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
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

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIMNS"stiffnessCoefficient"));
  c.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIMNS"dampingCoefficient"));
  d.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearRegularizedUnilateralConstraintProperty::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  d.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearRegularizedUnilateralConstraintProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
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

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0.01"),"m/s",MBSIMNS"marginalVelocity"));
  gd.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"frictionCoefficient"));
  mu.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearRegularizedCoulombFrictionProperty::initializeUsingXML(TiXmlElement *element) {
  gd.initializeUsingXML(element);
  mu.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearRegularizedCoulombFrictionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
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
