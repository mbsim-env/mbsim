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

//SymbolicFunction1Property::SymbolicFunction1Property(const string &ext, const string &var) : FunctionProperty(ext), argname(1), argdim(ext.size()-1) {
//  argname[0].setProperty(new TextProperty(var,""));
//  argdim[0].setProperty(new IntegerProperty(1,""));
//  f.setProperty(new OctaveExpressionProperty);
//}
//
//int SymbolicFunction1Property::getArg1Size() const {
//  return static_cast<const IntegerProperty*>(argdim[0].getProperty())->getValue();
//}
//
//TiXmlElement* SymbolicFunction1Property::initializeUsingXML(TiXmlElement *element) {
//  f.initializeUsingXML(element);
//  string str = "arg1name";
//  if(element->Attribute(str))
//    static_cast<TextProperty*>(argname[0].getProperty())->setText(element->Attribute(str.c_str()));
//  str = "arg1dim";
//  if(element->Attribute(str))
//    static_cast<IntegerProperty*>(argdim[0].getProperty())->setValue(atoi(element->Attribute(str.c_str())));
//  return element;
//}
//
//TiXmlElement* SymbolicFunction1Property::writeXMLFile(TiXmlNode *parent) {
//  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
//  ele0->SetAttribute("arg1name", static_cast<TextProperty*>(argname[0].getProperty())->getText());
//  if(ext[1]=='V')
//    ele0->SetAttribute("arg1dim",static_cast<IntegerProperty*>(argdim[0].getProperty())->getValue());
//  f.writeXMLFile(ele0);
//  return ele0;
//} 
//
//void SymbolicFunction1Property::fromWidget(QWidget *widget) {
//  argname[0].fromWidget(static_cast<SymbolicFunction1Widget*>(widget)->argname[0]);
//  argdim[0].fromWidget(static_cast<SymbolicFunction1Widget*>(widget)->argdim[0]);
//  f.fromWidget(static_cast<SymbolicFunction1Widget*>(widget)->f);
//}
//
//void SymbolicFunction1Property::toWidget(QWidget *widget) {
//  argname[0].toWidget(static_cast<SymbolicFunction1Widget*>(widget)->argname[0]);
//  argdim[0].toWidget(static_cast<SymbolicFunction1Widget*>(widget)->argdim[0]);
//  f.toWidget(static_cast<SymbolicFunction1Widget*>(widget)->f);
//}
//
//int SymbolicFunction1Property::getArgDim() const {
//  return static_cast<const IntegerProperty*>(argdim[0].getProperty())->getValue();
//}

ConstantFunctionProperty::ConstantFunctionProperty(const string &ext, int m) : FunctionProperty(ext) {
  vector<PhysicalVariableProperty> input;
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"value"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"value"));
  c.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* ConstantFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  c.initializeUsingXML(element);
  return element;
}

TiXmlElement* ConstantFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  c.writeXMLFile(ele0);
  return ele0;
} 

void ConstantFunctionProperty::fromWidget(QWidget *widget) {
  c.fromWidget(static_cast<ConstantFunctionWidget*>(widget)->c);
}

void ConstantFunctionProperty::toWidget(QWidget *widget) {
  c.toWidget(static_cast<ConstantFunctionWidget*>(widget)->c);
}

LinearFunctionProperty::LinearFunctionProperty(const string &ext, int m) : FunctionProperty(ext), b(0,false) {
  vector<PhysicalVariableProperty> input;
  if(ext[0]=='V')
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

TiXmlElement* LinearFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  a.initializeUsingXML(element);
  b.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  b.writeXMLFile(ele0);
  return ele0;
} 

void LinearFunctionProperty::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<LinearFunctionWidget*>(widget)->a);
  b.fromWidget(static_cast<LinearFunctionWidget*>(widget)->b);
}

void LinearFunctionProperty::toWidget(QWidget *widget) {
  a.toWidget(static_cast<LinearFunctionWidget*>(widget)->a);
  b.toWidget(static_cast<LinearFunctionWidget*>(widget)->b);
}


NestedFunctionProperty::NestedFunctionProperty(const string &ext, const vector<Property*> &property_) : FunctionProperty(ext) {
  fo.setProperty(new ChoiceProperty(MBSIMNS"outerFunction",property_));

  vector<Property*> property;
  vector<string> var;
  if(ext[1]=='V' and ext[2]=='S') {
    var.push_back("t");
    property.push_back(new SymbolicFunctionProperty("VS",var));
    property.push_back(new ConstantFunctionProperty("V"));
    property.push_back(new LinearFunctionProperty("V"));
    property.push_back(new QuadraticFunctionProperty("V"));
    property.push_back(new SinusFunctionProperty("V"));
  }
  else if(ext[1]=='S' and ext[2]=='V') {
    var.push_back("q");
    property.push_back(new SymbolicFunctionProperty("SV",var));
  }
  else if(ext[1]=='S' and ext[2]=='S') {
    var.push_back("t");
    property.push_back(new SymbolicFunctionProperty("SS",var));
    property.push_back(new LinearFunctionProperty("S"));
    property.push_back(new QuadraticFunctionProperty("S"));
    property.push_back(new SinusFunctionProperty("S"));
  }
  fi.setProperty(new ChoiceProperty(MBSIMNS"innerFunction",property));
}

int NestedFunctionProperty::getArg1Size() const {
  return ext[2]=='V'?static_cast<const FunctionProperty*>(static_cast<const ChoiceProperty*>(fi.getProperty())->getProperty())->getArg1Size():0;
}

TiXmlElement* NestedFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  fo.initializeUsingXML(element);
  fi.initializeUsingXML(element);
  return element;
}

TiXmlElement* NestedFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  fo.writeXMLFile(ele0);
  fi.writeXMLFile(ele0);
  return ele0;
} 

void NestedFunctionProperty::fromWidget(QWidget *widget) {
  fo.fromWidget(static_cast<NestedFunctionWidget*>(widget)->fo);
  fi.fromWidget(static_cast<NestedFunctionWidget*>(widget)->fi);
}

void NestedFunctionProperty::toWidget(QWidget *widget) {
  fo.toWidget(static_cast<NestedFunctionWidget*>(widget)->fo);
  fi.toWidget(static_cast<NestedFunctionWidget*>(widget)->fi);
}

VectorValuedFunctionProperty::VectorValuedFunctionProperty(const string &ext, int m) : FunctionProperty(ext), f(m) {
  for(int i=0; i<f.size(); i++) {
    vector<Property*> property;
    vector<string> var;
    var.push_back("t");
    property.push_back(new ConstantFunctionProperty("S"));
    property.push_back(new LinearFunctionProperty("S"));
    property.push_back(new QuadraticFunctionProperty("S"));
    property.push_back(new SinusFunctionProperty("S"));
    property.push_back(new SymbolicFunctionProperty("SS",var));
    f[i].setProperty(new ChoiceProperty("",property));
  }
}

TiXmlElement* VectorValuedFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e = element->FirstChildElement(MBSIMNS"function");
  for(int i=0; i<f.size(); i++) {
    f[i].initializeUsingXML(e);
    e=e->NextSiblingElement();
  }
  return element;
}

TiXmlElement* VectorValuedFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  for(int i=0; i<f.size(); i++) {
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"function");
    f[i].writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  return ele0;
} 

void VectorValuedFunctionProperty::fromWidget(QWidget *widget) {
  for(int i=0; i<f.size(); i++)
    f[i].fromWidget(static_cast<VectorValuedFunctionWidget*>(widget)->f[i]);
}

void VectorValuedFunctionProperty::toWidget(QWidget *widget) {
  for(int i=0; i<f.size(); i++)
    f[i].toWidget(static_cast<VectorValuedFunctionWidget*>(widget)->f[i]);
}

PiecewiseDefinedFunctionProperty::PiecewiseDefinedFunctionProperty(const std::string &ext) : FunctionProperty(ext), contDiff(0,false) {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"continouslyDifferentiable"));
  contDiff.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* PiecewiseDefinedFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  
  function.clear();
  TiXmlElement *e = element->FirstChildElement(MBSIMNS"function");
  while(e and e->ValueStr()==MBSIMNS"function") {
    function.push_back(ContainerProperty());
    vector<Property*> property;
    property.push_back(new ConstantFunctionProperty(ext));
    property.push_back(new QuadraticFunctionProperty(ext));
    property.push_back(new SinusFunctionProperty(ext));
    vector<string> var;
    var.push_back("t");
    property.push_back(new SymbolicFunctionProperty(ext+"S",var));
    function[function.size()-1].addProperty(new ChoiceProperty("",property));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(2),"-",MBSIMNS"limit"));
    function[function.size()-1].addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

    function[function.size()-1].initializeUsingXML(e);

    e=e->NextSiblingElement();
  }
  contDiff.initializeUsingXML(element);

  return element;
}

TiXmlElement* PiecewiseDefinedFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  for(int i=0; i<function.size(); i++) {
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"function");
    function[i].writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  contDiff.writeXMLFile(ele0);
  return ele0;
}

void PiecewiseDefinedFunctionProperty::fromWidget(QWidget *widget) {
  function.clear();
  for(unsigned int i=0; i<static_cast<PiecewiseDefinedFunctionWidget*>(widget)->stackedWidget->count(); i++) {
    function.push_back(ContainerProperty());

    vector<Property*> property;
    property.push_back(new ConstantFunctionProperty(ext));
    property.push_back(new QuadraticFunctionProperty(ext));
    property.push_back(new SinusFunctionProperty(ext));
    vector<string> var;
    var.push_back("t");
    property.push_back(new SymbolicFunctionProperty(ext+"S",var));
    function[function.size()-1].addProperty(new ChoiceProperty("",property));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(2),"-",MBSIMNS"limit"));
    function[function.size()-1].addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

    function[i].fromWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->stackedWidget->widget(i));
  }
  contDiff.fromWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->contDiff);
}

void PiecewiseDefinedFunctionProperty::toWidget(QWidget *widget) {
  for(unsigned int i=0; i<function.size(); i++) {
    static_cast<PiecewiseDefinedFunctionWidget*>(widget)->blockSignals(true);
    static_cast<PiecewiseDefinedFunctionWidget*>(widget)->addFunction();
    static_cast<PiecewiseDefinedFunctionWidget*>(widget)->blockSignals(false);
    function[i].toWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->stackedWidget->widget(i));
  }
  contDiff.toWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->contDiff);
}

LinearTranslationProperty::LinearTranslationProperty(const string &ext, int m, int n) : FunctionProperty(ext), b(0,false) {
  vector<PhysicalVariableProperty> input;
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new MatProperty(m,n),"",MBSIMNS"translationVectors"));
  else
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"translationVectors"));
  A.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"offset"));
  b.setProperty(new ExtPhysicalVarProperty(input));
}

int LinearTranslationProperty::getArg1Size() const {
  if(ext[0]=='V') {
    string str = evalOctaveExpression(static_cast<const ExtPhysicalVarProperty*>(A.getProperty())->getCurrentPhysicalVariableProperty().getValue());
    vector<vector<string> > A = strToMat(str);
    return A.size()?A[0].size():0;
  }
  return 0;
}

TiXmlElement* LinearTranslationProperty::initializeUsingXML(TiXmlElement *element) {
  A.initializeUsingXML(element);
  b.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearTranslationProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
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

QuadraticFunctionProperty::QuadraticFunctionProperty(const string &ext, int m) : FunctionProperty(ext) {

  vector<PhysicalVariableProperty> input;
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"a0"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"a0"));
  a0.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"a1"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"a1"));
  a1.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"a2"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"a2"));
  a2.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* QuadraticFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  a0.initializeUsingXML(element);
  a1.initializeUsingXML(element);
  a2.initializeUsingXML(element);
  return element;
}

TiXmlElement* QuadraticFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a0.writeXMLFile(ele0);
  a1.writeXMLFile(ele0);
  a2.writeXMLFile(ele0);
  return ele0;
}

void QuadraticFunctionProperty::fromWidget(QWidget *widget) {
  a0.fromWidget(static_cast<QuadraticFunctionWidget*>(widget)->a0);
  a1.fromWidget(static_cast<QuadraticFunctionWidget*>(widget)->a1);
  a2.fromWidget(static_cast<QuadraticFunctionWidget*>(widget)->a2);
}

void QuadraticFunctionProperty::toWidget(QWidget *widget) {
  a0.toWidget(static_cast<QuadraticFunctionWidget*>(widget)->a0);
  a1.toWidget(static_cast<QuadraticFunctionWidget*>(widget)->a1);
  a2.toWidget(static_cast<QuadraticFunctionWidget*>(widget)->a2);
}

SinusFunctionProperty::SinusFunctionProperty(const string &ext, int m) : FunctionProperty(ext) {

  vector<PhysicalVariableProperty> input;
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"amplitude"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"amplitude"));
  a.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"frequency"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"frequency"));
  f.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"phase"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"phase"));
  p.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  if(ext[0]=='V')
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",MBSIMNS"offset"));
  else
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"offset"));
  o.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* SinusFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  a.initializeUsingXML(element);
  f.initializeUsingXML(element);
  p.initializeUsingXML(element);
  o.initializeUsingXML(element);
  return element;
}

TiXmlElement* SinusFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  f.writeXMLFile(ele0);
  p.writeXMLFile(ele0);
  o.writeXMLFile(ele0);
  return ele0;
} 

void SinusFunctionProperty::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<SinusFunctionWidget*>(widget)->a);
  f.fromWidget(static_cast<SinusFunctionWidget*>(widget)->f);
  p.fromWidget(static_cast<SinusFunctionWidget*>(widget)->p);
  o.fromWidget(static_cast<SinusFunctionWidget*>(widget)->o);
}

void SinusFunctionProperty::toWidget(QWidget *widget) {
  a.toWidget(static_cast<SinusFunctionWidget*>(widget)->a);
  f.toWidget(static_cast<SinusFunctionWidget*>(widget)->f);
  p.toWidget(static_cast<SinusFunctionWidget*>(widget)->p);
  o.toWidget(static_cast<SinusFunctionWidget*>(widget)->o);
}

TabularFunctionProperty::TabularFunctionProperty() : choice("",vector<Property*>(),2) {

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

TiXmlElement* TabularFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  choice.initializeUsingXML(element);
  return element;
}

TiXmlElement* TabularFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  choice.writeXMLFile(ele0);
  return ele0;
} 

void TabularFunctionProperty::fromWidget(QWidget *widget) {
  choice.fromWidget(static_cast<TabularFunctionWidget*>(widget)->choice);
}

void TabularFunctionProperty::toWidget(QWidget *widget) {
  choice.toWidget(static_cast<TabularFunctionWidget*>(widget)->choice);
}

TiXmlElement* SummationFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  
  function.clear();
  TiXmlElement *e = element->FirstChildElement(MBSIMNS"function");
  while(e) {
    function.push_back(ContainerProperty());
    vector<Property*> property;
    property.push_back(new ConstantFunctionProperty("V"));
    property.push_back(new QuadraticFunctionProperty("V"));
    property.push_back(new SinusFunctionProperty("V"));
    property.push_back(new TabularFunctionProperty);
    property.push_back(new SummationFunctionProperty);
    vector<string> var;
    var.push_back("t");
    property.push_back(new SymbolicFunctionProperty("VS",var));
    function[function.size()-1].addProperty(new ChoiceProperty("",property));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMNS"factor"));
    function[function.size()-1].addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

    function[function.size()-1].initializeUsingXML(e);

    e=e->NextSiblingElement();
  }

  return element;
}

TiXmlElement* SummationFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  for(int i=0; i<function.size(); i++) {
    TiXmlElement *ele1 = new TiXmlElement(MBSIMNS"function");
    function[i].writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  return ele0;
}

void SummationFunctionProperty::fromWidget(QWidget *widget) {
  function.clear();
  for(unsigned int i=0; i<static_cast<SummationFunctionWidget*>(widget)->stackedWidget->count(); i++) {
    function.push_back(ContainerProperty());

    vector<Property*> property;
    property.push_back(new ConstantFunctionProperty("V"));
    property.push_back(new QuadraticFunctionProperty("V"));
    property.push_back(new SinusFunctionProperty("V"));
    property.push_back(new TabularFunctionProperty);
    property.push_back(new SummationFunctionProperty);
    vector<string> var;
    var.push_back("t");
    property.push_back(new SymbolicFunctionProperty("VS",var));
    function[function.size()-1].addProperty(new ChoiceProperty("",property));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMNS"factor"));
    function[function.size()-1].addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

    function[i].fromWidget(static_cast<SummationFunctionWidget*>(widget)->stackedWidget->widget(i));
  }
}

void SummationFunctionProperty::toWidget(QWidget *widget) {
  for(unsigned int i=0; i<function.size(); i++) {
    static_cast<SummationFunctionWidget*>(widget)->blockSignals(true);
    static_cast<SummationFunctionWidget*>(widget)->addFunction();
    static_cast<SummationFunctionWidget*>(widget)->blockSignals(false);
    function[i].toWidget(static_cast<SummationFunctionWidget*>(widget)->stackedWidget->widget(i));
  }
}

SymbolicFunctionProperty::SymbolicFunctionProperty(const string &ext, const vector<string> &var) : FunctionProperty(ext), argname(ext.size()-1), argdim(ext.size()-1) {
  for(int i=1; i<ext.size(); i++) {
     argname[i-1].setProperty(new TextProperty(var[i-1],""));
     argdim[i-1].setProperty(new IntegerProperty(1,""));
  }
  f.setProperty(new OctaveExpressionProperty);
}

TiXmlElement* SymbolicFunctionProperty::initializeUsingXML(TiXmlElement *element) {
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

TiXmlElement* SymbolicFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  for(int i=1; i<ext.size(); i++) {
    string istr = toStr(i);
    ele0->SetAttribute("arg"+istr+"name", static_cast<TextProperty*>(argname[i-1].getProperty())->getText());
    if(ext[i]=='V')
      ele0->SetAttribute("arg"+istr+"dim",static_cast<IntegerProperty*>(argdim[i-1].getProperty())->getValue());
  }
  f.writeXMLFile(ele0);
  return ele0;
} 

void SymbolicFunctionProperty::fromWidget(QWidget *widget) {
  for(int i=0; i<argname.size(); i++) {
    argname[i].fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->argname[i]);
    argdim[i].fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->argdim[i]);
  }
  f.fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->f);
}

void SymbolicFunctionProperty::toWidget(QWidget *widget) {
  for(int i=0; i<argname.size(); i++) {
    argname[i].toWidget(static_cast<SymbolicFunctionWidget*>(widget)->argname[i]);
    argdim[i].toWidget(static_cast<SymbolicFunctionWidget*>(widget)->argdim[i]);
  }
  f.toWidget(static_cast<SymbolicFunctionWidget*>(widget)->f);
}

int SymbolicFunctionProperty::getArg1Size() const {
  return static_cast<const IntegerProperty*>(argdim[0].getProperty())->getValue();
}

int SymbolicFunctionProperty::getArg2Size() const {
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
