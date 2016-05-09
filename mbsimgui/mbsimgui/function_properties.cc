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
#include "mainwindow.h"
#include <mbxmlutils/eval.h>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  class LimitedFunctionFunctionPropertyFactory : public PropertyFactory {
    public:
      LimitedFunctionFunctionPropertyFactory(PropertyFactory *factory_, const MBXMLUtils::FQN xmlName_) : factory(factory_), xmlName(xmlName_) { }
      Property* createProperty(int i=0);
    protected:
      PropertyFactory *factory;
      MBXMLUtils::FQN xmlName;
  };

  Property* LimitedFunctionFunctionPropertyFactory::createProperty(int i) {
    ContainerProperty *property = new ContainerProperty(MBSIM%"LimitedFunction");

    property->addProperty(new ExtProperty(new ChoiceProperty2(factory,xmlName,0),true,MBSIM%"function",false));

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"limit"));
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
  //  //FunctionPropertyFactory factory("VS",n,MBSIM%"function",0);
  //  FunctionPropertyFactory factory("PiecewiseDefinedFunction",n,"",0);
  //
  //  ContainerProperty *property = new ContainerProperty(MBSIM%"LimitedFunction");
  //  //property->addProperty(new ExtProperty(factory.createProperty(),true,"",false));
  //  property->addProperty(new ExtProperty(factory.createProperty(),true,MBSIM%"function",false));
  //
  //  vector<PhysicalVariableProperty> input;
  //  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"limit"));
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

  ConstantFunction::ConstantFunction(const string &name, Element *parent, int m) : Function(name,parent) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"a0"));
    a0.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* ConstantFunction::initializeUsingXML(DOMElement *element) {
    a0.initializeUsingXML(element);
    return element;
  }

  DOMElement* ConstantFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    a0.writeXMLFile(ele0);
    return ele0;
  } 

  void ConstantFunction::fromWidget(QWidget *widget) {
    a0.fromWidget(static_cast<ConstantFunctionWidget*>(widget)->a0);
  }

  void ConstantFunction::toWidget(QWidget *widget) {
    a0.toWidget(static_cast<ConstantFunctionWidget*>(widget)->a0);
  }

  LinearFunction::LinearFunction(const string &name, Element *parent, int m) : Function(name,parent), a0(0,false) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"a0"));
    a0.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("1"),"",MBSIM%"a1"));
    a1.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* LinearFunction::initializeUsingXML(DOMElement *element) {
    a0.initializeUsingXML(element);
    a1.initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    a0.writeXMLFile(ele0);
    a1.writeXMLFile(ele0);
    return ele0;
  } 

  void LinearFunction::fromWidget(QWidget *widget) {
    a0.fromWidget(static_cast<LinearFunctionWidget*>(widget)->a0);
    a1.fromWidget(static_cast<LinearFunctionWidget*>(widget)->a1);
  }

  void LinearFunction::toWidget(QWidget *widget) {
    a0.toWidget(static_cast<LinearFunctionWidget*>(widget)->a0);
    a1.toWidget(static_cast<LinearFunctionWidget*>(widget)->a1);
  }

  QuadraticFunction::QuadraticFunction(const string &name, Element *parent, int m) : Function(name,parent), a0(0,false), a1(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"a0"));
    a0.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"a1"));
    a1.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"a2"));
    a2.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* QuadraticFunction::initializeUsingXML(DOMElement *element) {
    a0.initializeUsingXML(element);
    a1.initializeUsingXML(element);
    a2.initializeUsingXML(element);
    return element;
  }

  DOMElement* QuadraticFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    a0.writeXMLFile(ele0);
    a1.writeXMLFile(ele0);
    a2.writeXMLFile(ele0);
    return ele0;
  }

  void QuadraticFunction::fromWidget(QWidget *widget) {
    a0.fromWidget(static_cast<QuadraticFunctionWidget*>(widget)->a0);
    a1.fromWidget(static_cast<QuadraticFunctionWidget*>(widget)->a1);
    a2.fromWidget(static_cast<QuadraticFunctionWidget*>(widget)->a2);
  }

  void QuadraticFunction::toWidget(QWidget *widget) {
    a0.toWidget(static_cast<QuadraticFunctionWidget*>(widget)->a0);
    a1.toWidget(static_cast<QuadraticFunctionWidget*>(widget)->a1);
    a2.toWidget(static_cast<QuadraticFunctionWidget*>(widget)->a2);
  }

  PolynomFunction::PolynomFunction(const string &name, Element *parent, int m) : Function(name,parent) {
    a.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"coefficients",vector<string>(3,"")),"",4));
  }

  DOMElement* PolynomFunction::initializeUsingXML(DOMElement *element) {
    a.initializeUsingXML(element);
    return element;
  }

  DOMElement* PolynomFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    a.writeXMLFile(ele0);
    return ele0;
  }

  void PolynomFunction::fromWidget(QWidget *widget) {
    a.fromWidget(static_cast<PolynomFunctionWidget*>(widget)->a);
  }

  void PolynomFunction::toWidget(QWidget *widget) {
    a.toWidget(static_cast<PolynomFunctionWidget*>(widget)->a);
  }

  SinusoidalFunction::SinusoidalFunction(const string &name, Element *parent, int m) : Function(name,parent), p(0,false), o(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"amplitude"));
    a.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"frequency"));
    f.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"phase"));
    p.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"offset"));
    o.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* SinusoidalFunction::initializeUsingXML(DOMElement *element) {
    a.initializeUsingXML(element);
    f.initializeUsingXML(element);
    p.initializeUsingXML(element);
    o.initializeUsingXML(element);
    return element;
  }

  DOMElement* SinusoidalFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    a.writeXMLFile(ele0);
    f.writeXMLFile(ele0);
    p.writeXMLFile(ele0);
    o.writeXMLFile(ele0);
    return ele0;
  } 

  void SinusoidalFunction::fromWidget(QWidget *widget) {
    a.fromWidget(static_cast<SinusoidalFunctionWidget*>(widget)->a);
    f.fromWidget(static_cast<SinusoidalFunctionWidget*>(widget)->f);
    p.fromWidget(static_cast<SinusoidalFunctionWidget*>(widget)->p);
    o.fromWidget(static_cast<SinusoidalFunctionWidget*>(widget)->o);
  }

  void SinusoidalFunction::toWidget(QWidget *widget) {
    a.toWidget(static_cast<SinusoidalFunctionWidget*>(widget)->a);
    f.toWidget(static_cast<SinusoidalFunctionWidget*>(widget)->f);
    p.toWidget(static_cast<SinusoidalFunctionWidget*>(widget)->p);
    o.toWidget(static_cast<SinusoidalFunctionWidget*>(widget)->o);
  }

  ModuloFunction::ModuloFunction(const string &name, Element *parent) : Function(name,parent) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"denominator"));
    denom.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* ModuloFunction::initializeUsingXML(DOMElement *element) {
    denom.initializeUsingXML(element);
    return element;
  }

  DOMElement* ModuloFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    denom.writeXMLFile(ele0);
    return ele0;
  } 

  void ModuloFunction::fromWidget(QWidget *widget) {
    denom.fromWidget(static_cast<ModuloFunctionWidget*>(widget)->denom);
  }

  void ModuloFunction::toWidget(QWidget *widget) {
    denom.toWidget(static_cast<ModuloFunctionWidget*>(widget)->denom);
  }

  AdditionFunction::AdditionFunction(const string &name, Element *parent) : Function(name,parent) {
    f1.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"firstSummand",0));
    f2.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"secondSummand",0));
  }

  DOMElement* AdditionFunction::initializeUsingXML(DOMElement *element) {
    f1.initializeUsingXML(element);
    f2.initializeUsingXML(element);
    return element;
  }

  DOMElement* AdditionFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    f1.writeXMLFile(ele0);
    f2.writeXMLFile(ele0);
    return ele0;
  }

  void AdditionFunction::fromWidget(QWidget *widget) {
    f1.fromWidget(static_cast<AdditionFunctionWidget*>(widget)->f1);
    f2.fromWidget(static_cast<AdditionFunctionWidget*>(widget)->f2);
  }

  void AdditionFunction::toWidget(QWidget *widget) {
    f1.toWidget(static_cast<AdditionFunctionWidget*>(widget)->f1);
    f2.toWidget(static_cast<AdditionFunctionWidget*>(widget)->f2);
  }

  MultiplicationFunction::MultiplicationFunction(const string &name, Element *parent) : Function(name,parent) {
    f1.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"firstFactor",0));
    f2.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"secondFactor",0));
  }

  DOMElement* MultiplicationFunction::initializeUsingXML(DOMElement *element) {
    f1.initializeUsingXML(element);
    f2.initializeUsingXML(element);
    return element;
  }

  DOMElement* MultiplicationFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    f1.writeXMLFile(ele0);
    f2.writeXMLFile(ele0);
    return ele0;
  }

  void MultiplicationFunction::fromWidget(QWidget *widget) {
    f1.fromWidget(static_cast<MultiplicationFunctionWidget*>(widget)->f1);
    f2.fromWidget(static_cast<MultiplicationFunctionWidget*>(widget)->f2);
  }

  void MultiplicationFunction::toWidget(QWidget *widget) {
    f1.toWidget(static_cast<MultiplicationFunctionWidget*>(widget)->f1);
    f2.toWidget(static_cast<MultiplicationFunctionWidget*>(widget)->f2);
  }

  VectorValuedFunction::VectorValuedFunction(const string &name, Element *parent, int m) : Function(name,parent) {
    functions.setProperty(new ListProperty(new ChoicePropertyFactory(new FunctionPropertyFactory2(this),""),""));
    functions.setXMLName(MBSIM%"components");
  }

  DOMElement* VectorValuedFunction::initializeUsingXML(DOMElement *element) {
    functions.initializeUsingXML(element);
    return element;
  }

  DOMElement* VectorValuedFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    functions.writeXMLFile(ele0);
    return ele0;
  }

  void VectorValuedFunction::fromWidget(QWidget *widget) {
    functions.fromWidget(static_cast<VectorValuedFunctionWidget*>(widget)->functions);
  }

  void VectorValuedFunction::toWidget(QWidget *widget) {
    functions.toWidget(static_cast<VectorValuedFunctionWidget*>(widget)->functions);
  }

  NestedFunction::NestedFunction(const string &name, Element *parent, PropertyFactory *factoryo, PropertyFactory *factoryi) : Function(name,parent) {
    fo.setProperty(new ChoiceProperty2(factoryo,MBSIM%"outerFunction",0));
    fi.setProperty(new ChoiceProperty2(factoryi,MBSIM%"innerFunction",0));
  }

  int NestedFunction::getArg1Size() const {
    //return static_cast<const FunctionProperty*>(static_cast<const ChoiceProperty*>(fi.getProperty())->getProperty())->getArg1Size();
    return 0;
  }

  DOMElement* NestedFunction::initializeUsingXML(DOMElement *element) {
    fo.initializeUsingXML(element);
    fi.initializeUsingXML(element);
    return element;
  }

  DOMElement* NestedFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    fo.writeXMLFile(ele0);
    fi.writeXMLFile(ele0);
    return ele0;
  } 

  void NestedFunction::fromWidget(QWidget *widget) {
    fo.fromWidget(static_cast<NestedFunctionWidget*>(widget)->fo);
    fi.fromWidget(static_cast<NestedFunctionWidget*>(widget)->fi);
  }

  void NestedFunction::toWidget(QWidget *widget) {
    fo.toWidget(static_cast<NestedFunctionWidget*>(widget)->fo);
    fi.toWidget(static_cast<NestedFunctionWidget*>(widget)->fi);
  }

  BinaryNestedFunction::BinaryNestedFunction(const string &name, Element *parent, PropertyFactory *factoryo, PropertyFactory *factoryi1, PropertyFactory *factoryi2) : Function(name,parent) {
    fo.setProperty(new ChoiceProperty2(factoryo,MBSIM%"outerFunction",0));
    fi1.setProperty(new ChoiceProperty2(factoryi1,MBSIM%"firstInnerFunction",0));
    fi2.setProperty(new ChoiceProperty2(factoryi2,MBSIM%"secondInnerFunction",0));
  }

  int BinaryNestedFunction::getArg1Size() const {
    //return static_cast<const FunctionProperty*>(static_cast<const ChoiceProperty*>(fi1.getProperty())->getProperty())->getArg1Size();
    return 0;
  }

  int BinaryNestedFunction::getArg2Size() const {
    //return static_cast<const FunctionProperty*>(static_cast<const ChoiceProperty*>(fi2.getProperty())->getProperty())->getArg1Size();
    return 0;
  }

  DOMElement* BinaryNestedFunction::initializeUsingXML(DOMElement *element) {
    fo.initializeUsingXML(element);
    fi1.initializeUsingXML(element);
    fi2.initializeUsingXML(element);
    return element;
  }

  DOMElement* BinaryNestedFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    fo.writeXMLFile(ele0);
    fi1.writeXMLFile(ele0);
    fi2.writeXMLFile(ele0);
    return ele0;
  }

  void BinaryNestedFunction::fromWidget(QWidget *widget) {
    fo.fromWidget(static_cast<BinaryNestedFunctionWidget*>(widget)->fo);
    fi1.fromWidget(static_cast<BinaryNestedFunctionWidget*>(widget)->fi1);
    fi2.fromWidget(static_cast<BinaryNestedFunctionWidget*>(widget)->fi2);
  }

  void BinaryNestedFunction::toWidget(QWidget *widget) {
    fo.toWidget(static_cast<BinaryNestedFunctionWidget*>(widget)->fo);
    fi1.toWidget(static_cast<BinaryNestedFunctionWidget*>(widget)->fi1);
    fi2.toWidget(static_cast<BinaryNestedFunctionWidget*>(widget)->fi2);
  }

  PiecewiseDefinedFunction::PiecewiseDefinedFunction(const string &name, Element *parent) : Function(name,parent) {
    //  functions.setProperty(new ListProperty(new LimitedFunctionFunctionPropertyFactory(1),""));
    //  functions.setXMLName(MBSIM%"limitedFunctions");
    functions.setProperty(new ListProperty(new LimitedFunctionFunctionPropertyFactory(new FunctionPropertyFactory2(this),""),""));
    functions.setXMLName(MBSIM%"limitedFunctions");
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"continouslyDifferentiable"));
    contDiff.setProperty(new ExtPhysicalVarProperty(input));
    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"shiftAbscissa"));
    shiftAbscissa.setProperty(new ExtPhysicalVarProperty(input));
    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"shiftOrdinate"));
    shiftOrdinate.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* PiecewiseDefinedFunction::initializeUsingXML(DOMElement *element) {
    functions.initializeUsingXML(element);
    contDiff.initializeUsingXML(element);
    shiftAbscissa.initializeUsingXML(element);
    shiftOrdinate.initializeUsingXML(element);
    return element;
  }

  DOMElement* PiecewiseDefinedFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    functions.writeXMLFile(ele0);
    contDiff.writeXMLFile(ele0);
    shiftAbscissa.writeXMLFile(ele0);
    shiftOrdinate.writeXMLFile(ele0);
    return ele0;
  }

  void PiecewiseDefinedFunction::fromWidget(QWidget *widget) {
    functions.fromWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->functions);
    contDiff.fromWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->contDiff);
    shiftAbscissa.fromWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->shiftAbscissa);
    shiftOrdinate.fromWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->shiftOrdinate);
  }

  void PiecewiseDefinedFunction::toWidget(QWidget *widget) {
    functions.toWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->functions);
    contDiff.toWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->contDiff);
    shiftAbscissa.toWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->shiftAbscissa);
    shiftOrdinate.toWidget(static_cast<PiecewiseDefinedFunctionWidget*>(widget)->shiftOrdinate);
  }

  SymbolicFunction::SymbolicFunction(const string &name, Element *parent, const string &ext_, const vector<string> &var, int m) : Function(name,parent), ext(ext_), argname(ext.size()-1), argdim(ext.size()-1) {
    for(size_t i=1; i<ext.size(); i++) {
      argname[i-1].setProperty(new TextProperty(var[i-1],""));
      argdim[i-1].setProperty(new IntegerProperty(1,""));
    }
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecProperty(m),"",""));
    f.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* SymbolicFunction::initializeUsingXML(DOMElement *element) {
    f.initializeUsingXML(element);
    for(int i=1; i<static_cast<int>(ext.size()); i++) {
      string str = "arg"+toStr(i);
      if(E(element)->hasAttribute(str))
        static_cast<TextProperty*>(argname[i-1].getProperty())->setText(E(element)->getAttribute(str.c_str()));
      str = "arg"+toStr(i)+"Dim";
      if(E(element)->hasAttribute(str))
        static_cast<IntegerProperty*>(argdim[i-1].getProperty())->setValue(atoi(E(element)->getAttribute(str.c_str()).c_str()));
    }
    return element;
  }

  DOMElement* SymbolicFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    for(int i=1; i<static_cast<int>(ext.size()); i++) {
      string istr = toStr(i);
      E(ele0)->setAttribute("arg"+istr, static_cast<TextProperty*>(argname[i-1].getProperty())->getText());
      if(ext[i]=='V')
        E(ele0)->setAttribute("arg"+istr+"Dim",boost::lexical_cast<string>(static_cast<IntegerProperty*>(argdim[i-1].getProperty())->getValue()));
    }
    f.writeXMLFile(ele0);
    return ele0;
  } 

  void SymbolicFunction::fromWidget(QWidget *widget) {
    for(size_t i=0; i<argname.size(); i++) {
      argname[i].fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->argname[i]);
      argdim[i].fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->argdim[i]);
    }
    f.fromWidget(static_cast<SymbolicFunctionWidget*>(widget)->f);
  }

  void SymbolicFunction::toWidget(QWidget *widget) {
    for(size_t i=0; i<argname.size(); i++) {
      argname[i].toWidget(static_cast<SymbolicFunctionWidget*>(widget)->argname[i]);
      argdim[i].toWidget(static_cast<SymbolicFunctionWidget*>(widget)->argdim[i]);
    }
    f.toWidget(static_cast<SymbolicFunctionWidget*>(widget)->f);
  }

  int SymbolicFunction::getArg1Size() const {
    return static_cast<const IntegerProperty*>(argdim[0].getProperty())->getValue();
  }

  int SymbolicFunction::getArg2Size() const {
    return static_cast<const IntegerProperty*>(argdim[1].getProperty())->getValue();
  }

  TabularFunction::TabularFunction(const string &name, Element *parent) : Function(name,parent), choice(new TabularFunctionPropertyFactory(this),"",3) {
  }

  DOMElement* TabularFunction::initializeUsingXML(DOMElement *element) {
    choice.initializeUsingXML(element);
    return element;
  }

  DOMElement* TabularFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    choice.writeXMLFile(ele0);
    return ele0;
  } 

  void TabularFunction::fromWidget(QWidget *widget) {
    choice.fromWidget(static_cast<TabularFunctionWidget*>(widget)->choice);
  }

  void TabularFunction::toWidget(QWidget *widget) {
    choice.toWidget(static_cast<TabularFunctionWidget*>(widget)->choice);
  }

  TwoDimensionalTabularFunction::TwoDimensionalTabularFunction(const string &name, Element *parent) : Function(name,parent) {
    x.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"xValues",vector<string>(3,"")),"",4));
    y.setProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"yValues",vector<string>(3,"")),"",4));
    xy.setProperty(new ChoiceProperty2(new MatPropertyFactory(getScalars<string>(3,1,"0"),MBSIM%"xyValues",vector<string>(3,"")),"",4));
  }

  DOMElement* TwoDimensionalTabularFunction::initializeUsingXML(DOMElement *element) {
    x.initializeUsingXML(element);
    y.initializeUsingXML(element);
    xy.initializeUsingXML(element);
    return element;
  }

  DOMElement* TwoDimensionalTabularFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    x.writeXMLFile(ele0);
    y.writeXMLFile(ele0);
    xy.writeXMLFile(ele0);
    return ele0;
  } 

  void TwoDimensionalTabularFunction::fromWidget(QWidget *widget) {
    x.fromWidget(static_cast<TwoDimensionalTabularFunctionWidget*>(widget)->x);
    y.fromWidget(static_cast<TwoDimensionalTabularFunctionWidget*>(widget)->y);
    xy.fromWidget(static_cast<TwoDimensionalTabularFunctionWidget*>(widget)->xy);
  }

  void TwoDimensionalTabularFunction::toWidget(QWidget *widget) {
    x.toWidget(static_cast<TwoDimensionalTabularFunctionWidget*>(widget)->x);
    y.toWidget(static_cast<TwoDimensionalTabularFunctionWidget*>(widget)->y);
    xy.toWidget(static_cast<TwoDimensionalTabularFunctionWidget*>(widget)->xy);
  }

  PiecewisePolynomFunction::PiecewisePolynomFunction(const string &name, Element *parent) : Function(name,parent), choice(new TabularFunctionPropertyFactory(this),"",3), method(0,false) {
    
    method.setProperty(new TextProperty("\"cSplineNatural\"", MBSIM%"interpolationMethod"));

  }

  DOMElement* PiecewisePolynomFunction::initializeUsingXML(DOMElement *element) {
    choice.initializeUsingXML(element);
    method.initializeUsingXML(element);
    return element;
  }

  DOMElement* PiecewisePolynomFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    choice.writeXMLFile(ele0);
    method.writeXMLFile(ele0);
    return ele0;
  } 

  void PiecewisePolynomFunction::fromWidget(QWidget *widget) {
    choice.fromWidget(static_cast<PiecewisePolynomFunctionWidget*>(widget)->choice);
    method.fromWidget(static_cast<PiecewisePolynomFunctionWidget*>(widget)->method);
  }

  void PiecewisePolynomFunction::toWidget(QWidget *widget) {
    choice.toWidget(static_cast<PiecewisePolynomFunctionWidget*>(widget)->choice);
    method.toWidget(static_cast<PiecewisePolynomFunctionWidget*>(widget)->method);
  }

  FourierFunction::FourierFunction(const string &name, Element *parent) : Function(name,parent), a0(0,false), amplitudePhaseAngleForm(0,false), choice(new FourierFunctionPropertyFactory(this),"",3) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"frequency"));
    f.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"",MBSIM%"a0"));
    a0.setProperty(new ExtPhysicalVarProperty(input));
    amplitudePhaseAngleForm.setProperty(new ChoiceProperty2(new ScalarPropertyFactory("1",MBSIM%"amplitudePhaseAngleForm",vector<string>(2,"")),"",4));
  }

  DOMElement* FourierFunction::initializeUsingXML(DOMElement *element) {
    f.initializeUsingXML(element);
    a0.initializeUsingXML(element);
    choice.initializeUsingXML(element);
    amplitudePhaseAngleForm.initializeUsingXML(element);
    return element;
  }

  DOMElement* FourierFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    f.writeXMLFile(ele0);
    a0.writeXMLFile(ele0);
    choice.writeXMLFile(ele0);
    amplitudePhaseAngleForm.writeXMLFile(ele0);
    return ele0;
  } 

  void FourierFunction::fromWidget(QWidget *widget) {
    f.fromWidget(static_cast<FourierFunctionWidget*>(widget)->f);
    a0.fromWidget(static_cast<FourierFunctionWidget*>(widget)->a0);
    choice.fromWidget(static_cast<FourierFunctionWidget*>(widget)->choice);
    amplitudePhaseAngleForm.fromWidget(static_cast<FourierFunctionWidget*>(widget)->amplitudePhaseAngleForm);
  }

  void FourierFunction::toWidget(QWidget *widget) {
    f.toWidget(static_cast<FourierFunctionWidget*>(widget)->f);
    a0.toWidget(static_cast<FourierFunctionWidget*>(widget)->a0);
    choice.toWidget(static_cast<FourierFunctionWidget*>(widget)->choice);
    amplitudePhaseAngleForm.toWidget(static_cast<FourierFunctionWidget*>(widget)->amplitudePhaseAngleForm);
  }

  LinearSpringDamperForce::LinearSpringDamperForce(const string &name, Element *parent) : Function(name,parent) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIM%"stiffnessCoefficient"));
    c.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIM%"dampingCoefficient"));
    d.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"m",MBSIM%"unloadedLength"));
    l0.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* LinearSpringDamperForce::initializeUsingXML(DOMElement *element) {
    c.initializeUsingXML(element);
    d.initializeUsingXML(element);
    l0.initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearSpringDamperForce::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    c.writeXMLFile(ele0);
    d.writeXMLFile(ele0);
    l0.writeXMLFile(ele0);
    return ele0;
  } 

  void LinearSpringDamperForce::fromWidget(QWidget *widget) {
    c.fromWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->c);
    d.fromWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->d);
    l0.fromWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->l0);
  }

  void LinearSpringDamperForce::toWidget(QWidget *widget) {
    c.toWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->c);
    d.toWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->d);
    l0.toWidget(static_cast<LinearSpringDamperForceWidget*>(widget)->l0);
  }

  NonlinearSpringDamperForce::NonlinearSpringDamperForce(const string &name, Element *parent) : Function(name,parent) {

    g.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"distanceForce",0));

    gd.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"velocityForce",0));
  }

  DOMElement* NonlinearSpringDamperForce::initializeUsingXML(DOMElement *element) {
    g.initializeUsingXML(element);
    gd.initializeUsingXML(element);
    return element;
  }

  DOMElement* NonlinearSpringDamperForce::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    g.writeXMLFile(ele0);
    gd.writeXMLFile(ele0);
    return ele0;
  } 

  void NonlinearSpringDamperForce::fromWidget(QWidget *widget) {
    g.fromWidget(static_cast<NonlinearSpringDamperForceWidget*>(widget)->g);
    gd.fromWidget(static_cast<NonlinearSpringDamperForceWidget*>(widget)->gd);
  }

  void NonlinearSpringDamperForce::toWidget(QWidget *widget) {
    g.toWidget(static_cast<NonlinearSpringDamperForceWidget*>(widget)->g);
    gd.toWidget(static_cast<NonlinearSpringDamperForceWidget*>(widget)->gd);
  }

  LinearRegularizedBilateralConstraint::LinearRegularizedBilateralConstraint(const string &name, Element *parent) : Function(name,parent) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIM%"stiffnessCoefficient"));
    c.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIM%"dampingCoefficient"));
    d.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* LinearRegularizedBilateralConstraint::initializeUsingXML(DOMElement *element) {
    c.initializeUsingXML(element);
    d.initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedBilateralConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    c.writeXMLFile(ele0);
    d.writeXMLFile(ele0);
    return ele0;
  } 

  void LinearRegularizedBilateralConstraint::fromWidget(QWidget *widget) {
    c.fromWidget(static_cast<LinearRegularizedBilateralConstraintWidget*>(widget)->c);
    d.fromWidget(static_cast<LinearRegularizedBilateralConstraintWidget*>(widget)->d);
  }

  void LinearRegularizedBilateralConstraint::toWidget(QWidget *widget) {
    c.toWidget(static_cast<LinearRegularizedBilateralConstraintWidget*>(widget)->c);
    d.toWidget(static_cast<LinearRegularizedBilateralConstraintWidget*>(widget)->d);
  }

  LinearRegularizedUnilateralConstraint::LinearRegularizedUnilateralConstraint(const string &name, Element *parent) : Function(name,parent) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N/m",MBSIM%"stiffnessCoefficient"));
    c.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"N*s/m",MBSIM%"dampingCoefficient"));
    d.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* LinearRegularizedUnilateralConstraint::initializeUsingXML(DOMElement *element) {
    c.initializeUsingXML(element);
    d.initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedUnilateralConstraint::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    c.writeXMLFile(ele0);
    d.writeXMLFile(ele0);
    return ele0;
  } 

  void LinearRegularizedUnilateralConstraint::fromWidget(QWidget *widget) {
    c.fromWidget(static_cast<LinearRegularizedUnilateralConstraintWidget*>(widget)->c);
    d.fromWidget(static_cast<LinearRegularizedUnilateralConstraintWidget*>(widget)->d);
  }

  void LinearRegularizedUnilateralConstraint::toWidget(QWidget *widget) {
    c.toWidget(static_cast<LinearRegularizedUnilateralConstraintWidget*>(widget)->c);
    d.toWidget(static_cast<LinearRegularizedUnilateralConstraintWidget*>(widget)->d);
  }

  LinearRegularizedCoulombFriction::LinearRegularizedCoulombFriction(const string &name, Element *parent) : Function(name,parent), gd(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0.01"),"m/s",MBSIM%"marginalVelocity"));
    gd.setProperty(new ExtPhysicalVarProperty(input));

    input.clear();
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
    mu.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* LinearRegularizedCoulombFriction::initializeUsingXML(DOMElement *element) {
    gd.initializeUsingXML(element);
    mu.initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedCoulombFriction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    gd.writeXMLFile(ele0);
    mu.writeXMLFile(ele0);
    return ele0;
  } 

  void LinearRegularizedCoulombFriction::fromWidget(QWidget *widget) {
    gd.fromWidget(static_cast<LinearRegularizedCoulombFrictionWidget*>(widget)->gd);
    mu.fromWidget(static_cast<LinearRegularizedCoulombFrictionWidget*>(widget)->mu);
  }

  void LinearRegularizedCoulombFriction::toWidget(QWidget *widget) {
    gd.toWidget(static_cast<LinearRegularizedCoulombFrictionWidget*>(widget)->gd);
    mu.toWidget(static_cast<LinearRegularizedCoulombFrictionWidget*>(widget)->mu);
  }

  LinearRegularizedStribeckFriction::LinearRegularizedStribeckFriction(const string &name, Element *parent) : Function(name,parent), gd(0,false) {

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0.01"),"m/s",MBSIM%"marginalVelocity"));
    gd.setProperty(new ExtPhysicalVarProperty(input));

    mu.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"frictionFunction",0));
  }

  DOMElement* LinearRegularizedStribeckFriction::initializeUsingXML(DOMElement *element) {
    gd.initializeUsingXML(element);
    mu.initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedStribeckFriction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    gd.writeXMLFile(ele0);
    mu.writeXMLFile(ele0);
    return ele0;
  }

  void LinearRegularizedStribeckFriction::fromWidget(QWidget *widget) {
    gd.fromWidget(static_cast<LinearRegularizedStribeckFrictionWidget*>(widget)->gd);
    mu.fromWidget(static_cast<LinearRegularizedStribeckFrictionWidget*>(widget)->mu);
  }

  void LinearRegularizedStribeckFriction::toWidget(QWidget *widget) {
    gd.toWidget(static_cast<LinearRegularizedStribeckFrictionWidget*>(widget)->gd);
    mu.toWidget(static_cast<LinearRegularizedStribeckFrictionWidget*>(widget)->mu);
  }

  SignalFunction::SignalFunction(const string &name, Element *parent) : Function(name,parent) {

    sRef.setProperty(new SignalOfReferenceProperty("",this,MBSIMCONTROL%"returnSignal"));

  }

  DOMElement* SignalFunction::initializeUsingXML(DOMElement *element) {
    sRef.initializeUsingXML(element);
    return element;
  }

  DOMElement* SignalFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    sRef.writeXMLFile(ele0);
    return ele0;
  } 

  void SignalFunction::fromWidget(QWidget *widget) {
    sRef.fromWidget(static_cast<SignalFunctionWidget*>(widget)->sRef);
  }

  void SignalFunction::toWidget(QWidget *widget) {
    sRef.toWidget(static_cast<SignalFunctionWidget*>(widget)->sRef);
  }

  PolarContourFunction::PolarContourFunction(const string &name, Element *parent) : Function(name,parent) {
    radiusFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"radiusFunction"));
  }

  DOMElement* PolarContourFunction::initializeUsingXML(DOMElement *element) {
    radiusFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* PolarContourFunction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Function::writeXMLFile(parent);
    radiusFunction.writeXMLFile(ele0);
    return ele0;
  }

  void PolarContourFunction::fromWidget(QWidget *widget) {
    radiusFunction.fromWidget(static_cast<PolarContourFunctionWidget*>(widget)->radiusFunction);
  }

  void PolarContourFunction::toWidget(QWidget *widget) {
    radiusFunction.toWidget(static_cast<PolarContourFunctionWidget*>(widget)->radiusFunction);
  }

}
