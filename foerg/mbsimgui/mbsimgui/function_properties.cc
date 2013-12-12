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
#include "function_property_factory.h"
#include <QSpinBox>
#include "mainwindow.h"
#include <mbxmlutils/octeval.h>

using namespace std;
using namespace MBXMLUtils;

class LimitedFunctionFunctionPropertyFactory : public PropertyFactory {
  public:
    LimitedFunctionFunctionPropertyFactory(PropertyFactory *factory_, const std::string xmlName_) : factory(factory_), xmlName(xmlName_) { }
    Property* createProperty(int i=0);
  protected:
    PropertyFactory *factory;
    std::string xmlName;
};

Property* LimitedFunctionFunctionPropertyFactory::createProperty(int i) {
  ContainerProperty *property = new ContainerProperty(MBSIMNS"LimitedFunction");

  property->addProperty(new ExtProperty(new ChoiceProperty2(factory,xmlName,0),true,MBSIMNS"function",false));

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"limit"));
  property->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));
  return property;
}

//class LimitedFunctionFunctionPropertyFactory : public PropertyFactory {
//  public:
//    LimitedFunctionFunctionPropertyFactory(int n_) : n(n_) { }
//    Property* createProperty(int i=0);
//  protected:
//    int n;
//};
//
//Property* LimitedFunctionFunctionPropertyFactory::createProperty(int i) {
//  //FunctionPropertyFactory factory("VS",n,MBSIMNS"function",0);
//  FunctionPropertyFactory factory("PiecewiseDefinedFunction",n,"",0);
//
//  ContainerProperty *property = new ContainerProperty(MBSIMNS"LimitedFunction");
//  //property->addProperty(new ExtProperty(factory.createProperty(),true,"",false));
//  property->addProperty(new ExtProperty(factory.createProperty(),true,MBSIMNS"function",false));
//
//  vector<PhysicalVariableProperty> input;
//  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"limit"));
//  property->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));
//
//  return property;
//}

class CoefficientPropertyFactory : public PropertyFactory {
  public:
    CoefficientPropertyFactory() { }
    Property* createProperty(int i=0);
};

Property* CoefficientPropertyFactory::createProperty(int i) {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",""));
  return new ExtPhysicalVarProperty(input);
}

ConstantFunctionProperty::ConstantFunctionProperty(int m) {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"a0"));
  a0.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* ConstantFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  a0.initializeUsingXML(element);
  return element;
}

TiXmlElement* ConstantFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a0.writeXMLFile(ele0);
  return ele0;
} 

void ConstantFunctionProperty::fromWidget(QWidget *widget) {
  a0.fromWidget(static_cast<ConstantFunctionWidget*>(widget)->a0);
}

void ConstantFunctionProperty::toWidget(QWidget *widget) {
  a0.toWidget(static_cast<ConstantFunctionWidget*>(widget)->a0);
}

LinearFunctionProperty::LinearFunctionProperty(int m) : a0(0,false) {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"a0"));
  a0.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"",MBSIMNS"a1"));
  a1.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* LinearFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  a0.initializeUsingXML(element);
  a1.initializeUsingXML(element);
  return element;
}

TiXmlElement* LinearFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a0.writeXMLFile(ele0);
  a1.writeXMLFile(ele0);
  return ele0;
} 

void LinearFunctionProperty::fromWidget(QWidget *widget) {
  a0.fromWidget(static_cast<LinearFunctionWidget*>(widget)->a0);
  a1.fromWidget(static_cast<LinearFunctionWidget*>(widget)->a1);
}

void LinearFunctionProperty::toWidget(QWidget *widget) {
  a0.toWidget(static_cast<LinearFunctionWidget*>(widget)->a0);
  a1.toWidget(static_cast<LinearFunctionWidget*>(widget)->a1);
}

QuadraticFunctionProperty::QuadraticFunctionProperty(int m) : a0(0,false), a1(0,false) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"a0"));
  a0.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"a1"));
  a1.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
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

PolynomFunctionProperty::PolynomFunctionProperty(int m) {
  a.setProperty(new ListProperty(new CoefficientPropertyFactory,MBSIMNS"coefficient"));
  a.setXMLName(MBSIMNS"coefficients");
}

TiXmlElement* PolynomFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  a.initializeUsingXML(element);
  return element;
}

TiXmlElement* PolynomFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  return ele0;
}

void PolynomFunctionProperty::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<PolynomFunctionWidget*>(widget)->a);
}

void PolynomFunctionProperty::toWidget(QWidget *widget) {
  a.toWidget(static_cast<PolynomFunctionWidget*>(widget)->a);
}

SinusoidalFunctionProperty::SinusoidalFunctionProperty(int m) : p(0,false), o(0,false) {

  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"amplitude"));
  a.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"frequency"));
  f.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"phase"));
  p.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIMNS"offset"));
  o.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* SinusoidalFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  a.initializeUsingXML(element);
  f.initializeUsingXML(element);
  p.initializeUsingXML(element);
  o.initializeUsingXML(element);
  return element;
}

TiXmlElement* SinusoidalFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  a.writeXMLFile(ele0);
  f.writeXMLFile(ele0);
  p.writeXMLFile(ele0);
  o.writeXMLFile(ele0);
  return ele0;
} 

void SinusoidalFunctionProperty::fromWidget(QWidget *widget) {
  a.fromWidget(static_cast<SinusoidalFunctionWidget*>(widget)->a);
  f.fromWidget(static_cast<SinusoidalFunctionWidget*>(widget)->f);
  p.fromWidget(static_cast<SinusoidalFunctionWidget*>(widget)->p);
  o.fromWidget(static_cast<SinusoidalFunctionWidget*>(widget)->o);
}

void SinusoidalFunctionProperty::toWidget(QWidget *widget) {
  a.toWidget(static_cast<SinusoidalFunctionWidget*>(widget)->a);
  f.toWidget(static_cast<SinusoidalFunctionWidget*>(widget)->f);
  p.toWidget(static_cast<SinusoidalFunctionWidget*>(widget)->p);
  o.toWidget(static_cast<SinusoidalFunctionWidget*>(widget)->o);
}

ScaledFunctionProperty::ScaledFunctionProperty() : factor(0,false) {
  function.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2,MBSIMNS"function",0));
  
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"-",MBSIMNS"scalingFactor"));
  factor.setProperty(new ExtPhysicalVarProperty(input));
}

int ScaledFunctionProperty::getArg1Size() const {
  return static_cast<const FunctionProperty*>(static_cast<const ChoiceProperty2*>(function.getProperty())->getProperty())->getArg1Size();
}

TiXmlElement* ScaledFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  function.initializeUsingXML(element);
  factor.initializeUsingXML(element);
  return element;
}

TiXmlElement* ScaledFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  function.writeXMLFile(ele0);
  factor.writeXMLFile(ele0);
  return ele0;
} 

void ScaledFunctionProperty::fromWidget(QWidget *widget) {
  function.fromWidget(static_cast<ScaledFunctionWidget*>(widget)->function);
  factor.fromWidget(static_cast<ScaledFunctionWidget*>(widget)->factor);
}

void ScaledFunctionProperty::toWidget(QWidget *widget) {
  function.toWidget(static_cast<ScaledFunctionWidget*>(widget)->function);
  factor.toWidget(static_cast<ScaledFunctionWidget*>(widget)->factor);
}

SummationFunctionProperty::SummationFunctionProperty() {
//  functions.setProperty(new ListProperty(new FunctionPropertyFactory("SummationFunction",1),""));
  functions.setProperty(new ListProperty(new ChoicePropertyFactory(new FunctionPropertyFactory2,""),""));
  functions.setXMLName(MBSIMNS"summands");
}

TiXmlElement* SummationFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  functions.initializeUsingXML(element);
  return element;
}

TiXmlElement* SummationFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  functions.writeXMLFile(ele0);
  return ele0;
}

void SummationFunctionProperty::fromWidget(QWidget *widget) {
  functions.fromWidget(static_cast<SummationFunctionWidget*>(widget)->functions);
}

void SummationFunctionProperty::toWidget(QWidget *widget) {
  functions.toWidget(static_cast<SummationFunctionWidget*>(widget)->functions);
}

VectorValuedFunctionProperty::VectorValuedFunctionProperty(int m) {
//  functions.setProperty(new ListProperty(new FunctionPropertyFactory("VectorValuedFunction",1),""));
//  functions.setXMLName(MBSIMNS"components");
  functions.setProperty(new ListProperty(new ChoicePropertyFactory(new FunctionPropertyFactory2,""),""));
  functions.setXMLName(MBSIMNS"components");
}

TiXmlElement* VectorValuedFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  functions.initializeUsingXML(element);
  return element;
}

TiXmlElement* VectorValuedFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  functions.writeXMLFile(ele0);
  return ele0;
}

void VectorValuedFunctionProperty::fromWidget(QWidget *widget) {
  functions.fromWidget(static_cast<VectorValuedFunctionWidget*>(widget)->functions);
}

void VectorValuedFunctionProperty::toWidget(QWidget *widget) {
  functions.toWidget(static_cast<VectorValuedFunctionWidget*>(widget)->functions);
}

//NestedFunctionProperty::NestedFunctionProperty(const string &ext, const vector<Property*> &property_) {
//  fo.setProperty(new ChoiceProperty(MBSIMNS"outerFunction",property_));
//
//  vector<Property*> property;
//  vector<string> var;
//  if(ext[1]=='V' and ext[2]=='V') {
//    var.push_back("q");
//    property.push_back(new SymbolicFunctionProperty("VV",var));
//  }
//  else if(ext[1]=='V' and ext[2]=='S') {
//    var.push_back("t");
//    property.push_back(new SymbolicFunctionProperty("VS",var));
//    property.push_back(new ConstantFunctionProperty);
//    property.push_back(new LinearFunctionProperty);
//    property.push_back(new QuadraticFunctionProperty);
//    property.push_back(new SinusoidalFunctionProperty);
//  }
//  fi.setProperty(new ChoiceProperty(MBSIMNS"innerFunction",property));
//}

NestedFunctionProperty::NestedFunctionProperty(PropertyFactory *factoryo, PropertyFactory *factoryi) {
  fo.setProperty(new ChoiceProperty2(factoryo,MBSIMNS"outerFunction",0));
  fi.setProperty(new ChoiceProperty2(factoryi,MBSIMNS"innerFunction",0));
}

int NestedFunctionProperty::getArg1Size() const {
  //return static_cast<const FunctionProperty*>(static_cast<const ChoiceProperty*>(fi.getProperty())->getProperty())->getArg1Size();
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

PiecewiseDefinedFunctionProperty::PiecewiseDefinedFunctionProperty() {
//  functions.setProperty(new ListProperty(new LimitedFunctionFunctionPropertyFactory(1),""));
//  functions.setXMLName(MBSIMNS"limitedFunctions");
  functions.setProperty(new ListProperty(new LimitedFunctionFunctionPropertyFactory(new FunctionPropertyFactory2,""),""));
  functions.setXMLName(MBSIMNS"limitedFunctions");
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"continouslyDifferentiable"));
  contDiff.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* PiecewiseDefinedFunctionProperty::initializeUsingXML(TiXmlElement *element) {
  functions.initializeUsingXML(element);
  contDiff.initializeUsingXML(element);
  return element;
}

TiXmlElement* PiecewiseDefinedFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  functions.writeXMLFile(ele0);
  contDiff.writeXMLFile(ele0);
  return ele0;
}

void PiecewiseDefinedFunctionProperty::fromWidget(QWidget *widget) {
  functions.fromWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->functions);
  contDiff.fromWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->contDiff);
}

void PiecewiseDefinedFunctionProperty::toWidget(QWidget *widget) {
  functions.toWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->functions);
  contDiff.toWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->contDiff);
}

SymbolicFunctionProperty::SymbolicFunctionProperty(const string &name, const vector<string> &var, int m) : FunctionProperty(name), argname(var.size()), argdim(var.size()) {
  for(int i=0; i<var.size(); i++) {
     argname[i] = var[i];
     argdim[i] = new IntegerProperty("",1);
  }
  f = new VecProperty(3,Units());
}

TiXmlElement* SymbolicFunctionProperty::initializeUsingXML(TiXmlElement *element) {
//  f.initializeUsingXML(element);
//  for(int i=1; i<ext.size(); i++) {
//    string str = "arg"+toStr(i)+"name";
//    if(element->Attribute(str))
//      static_cast<TextProperty*>(argname[i-1].getProperty())->setValue(element->Attribute(str.c_str()));
//    str = "arg"+toStr(i)+"dim";
//    if(element->Attribute(str))
//      static_cast<IntegerProperty*>(argdim[i-1].getProperty())->setInt(atoi(element->Attribute(str.c_str())));
//  }
//  return element;
}

TiXmlElement* SymbolicFunctionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = FunctionProperty::writeXMLFile(parent);
  for(int i=0; i<argname.size(); i++) {
    string istr = toStr(i+1);
    ele0->SetAttribute("arg"+istr+"name", argname[i]);
    if(argname[i]!="t")
      ele0->SetAttribute("arg"+istr+"dim",toStr(argdim[i]->getInt()));
  }
  f->writeXMLFile(ele0);
  return ele0;
} 

void SymbolicFunctionProperty::fromWidget(QWidget *widget) {
//  for(int i=0; i<argname.size(); i++) {
//    argname[i].fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->argname[i]);
//    argdim[i].fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->argdim[i]);
//  }
//  f.fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->f);
}

void SymbolicFunctionProperty::toWidget(QWidget *widget) {
//  for(int i=0; i<argname.size(); i++) {
//    argname[i].toWidget(static_cast<SymbolicFunctionWidget*>(widget)->argname[i]);
//    argdim[i].toWidget(static_cast<SymbolicFunctionWidget*>(widget)->argdim[i]);
//  }
//  f.toWidget(static_cast<SymbolicFunctionWidget*>(widget)->f);
}

int SymbolicFunctionProperty::getArg1Size() const {
//  return static_cast<const IntegerProperty*>(argdim[0].getProperty())->getInt();
}

int SymbolicFunctionProperty::getArg2Size() const {
//  return static_cast<const IntegerProperty*>(argdim[1].getProperty())->getInt();
}

Widget* SymbolicFunctionProperty::createWidget() {
  QStringList var;
  for(int i=0; i<argname.size(); i++)
   var << QString::fromStdString(argname[i]);
  return new SymbolicFunctionWidget(this,var,1,10);
}

TabularFunctionProperty::TabularFunctionProperty() : choice(new TabularFunctionPropertyFactory,"",3) {
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
