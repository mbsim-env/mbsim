/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "basic_widgets.h"
#include "function_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "utils.h"
#include "function.h"
#include "function_widget_factory.h"
#include "mainwindow.h"
#include "signal_.h"
#include <mbxmlutils/eval.h>
#include <fmatvec/toString.h>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  class LimitedFunctionWidgetFactory : public WidgetFactory {
    public:
      LimitedFunctionWidgetFactory(WidgetFactory *factory_) : factory(factory_) { }
      QString getName(int i=0) const override { return "Limited function"; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return MBSIM%"LimitedFunction"; }
      Widget* createWidget(int i=0) override;
      int getSize() const override { return 1; }
    protected:
      WidgetFactory *factory;
  };

  Widget* LimitedFunctionWidgetFactory::createWidget(int i) {
    if(i==0)
      return new LimitedFunctionWidget(factory);
    return nullptr;
  }

  class CoefficientWidgetFactory : public WidgetFactory {
    public:
      CoefficientWidgetFactory() = default;
      Widget* createWidget(int i=0) override;
  };

  Widget* CoefficientWidgetFactory::createWidget(int i) {
    return new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5);
  }

  ConstantFunctionWidget::ConstantFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    a0 = new ExtWidget("a0",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"a0");
    layout->addWidget(a0);
  }

  DOMElement* ConstantFunctionWidget::initializeUsingXML(DOMElement *element) {
    a0->initializeUsingXML(element);
    return element;
  }

  DOMElement* ConstantFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    a0->writeXMLFile(ele0);
    return ele0;
  }

  LinearFunctionWidget::LinearFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a0 = new ExtWidget("a0",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a0");
    layout->addWidget(a0);

    a1 = new ExtWidget("a1",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"a1");
    layout->addWidget(a1);
  }

  DOMElement* LinearFunctionWidget::initializeUsingXML(DOMElement *element) {
    a0->initializeUsingXML(element);
    a1->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    a0->writeXMLFile(ele0);
    a1->writeXMLFile(ele0);
    return ele0;
  }

  QuadraticFunctionWidget::QuadraticFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a0 = new ExtWidget("a0",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a0");
    layout->addWidget(a0);

    a1 = new ExtWidget("a1",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a1");
    layout->addWidget(a1);

    a2 = new ExtWidget("a2",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"a2");
    layout->addWidget(a2);
  }

  DOMElement* QuadraticFunctionWidget::initializeUsingXML(DOMElement *element) {
    a0->initializeUsingXML(element);
    a1->initializeUsingXML(element);
    a2->initializeUsingXML(element);
    return element;
  }

  DOMElement* QuadraticFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    a0->writeXMLFile(ele0);
    a1->writeXMLFile(ele0);
    a2->writeXMLFile(ele0);
    return ele0;
  }

  PolynomFunctionWidget::PolynomFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a = new ExtWidget("Coefficients",new ChoiceWidget(new VecSizeVarWidgetFactory(3,1,100,1,vector<QStringList>(3,QStringList()),vector<int>(3,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"coefficients");
    layout->addWidget(a);
  }

  DOMElement* PolynomFunctionWidget::initializeUsingXML(DOMElement *element) {
    a->initializeUsingXML(element);
    return element;
  }

  DOMElement* PolynomFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    a->writeXMLFile(ele0);
    return ele0;
  }

  SinusoidalFunctionWidget::SinusoidalFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a = new ExtWidget("Amplitude",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"amplitude");
    layout->addWidget(a);

    f = new ExtWidget("Frequency",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frequency");
    layout->addWidget(f);

    p = new ExtWidget("Phase",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"phase");
    layout->addWidget(p);

    o = new ExtWidget("Offset",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"offset");
    layout->addWidget(o);
  }

  DOMElement* SinusoidalFunctionWidget::initializeUsingXML(DOMElement *element) {
    a->initializeUsingXML(element);
    f->initializeUsingXML(element);
    p->initializeUsingXML(element);
    o->initializeUsingXML(element);
    return element;
  }

  DOMElement* SinusoidalFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    a->writeXMLFile(ele0);
    f->writeXMLFile(ele0);
    p->writeXMLFile(ele0);
    o->writeXMLFile(ele0);
    return ele0;
  }

  ModuloFunctionWidget::ModuloFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    denom = new ExtWidget("Denominator",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"denominator");
    layout->addWidget(denom);
  }

  DOMElement* ModuloFunctionWidget::initializeUsingXML(DOMElement *element) {
    denom->initializeUsingXML(element);
    return element;
  }

  DOMElement* ModuloFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    denom->writeXMLFile(ele0);
    return ele0;
  }

  BoundedFunctionWidget::BoundedFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    lowerBound = new ExtWidget("Lower bound",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"lowerBound");
    layout->addWidget(lowerBound);

    upperBound = new ExtWidget("Upper bound",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"upperBound");
    layout->addWidget(upperBound);
  }

  DOMElement* BoundedFunctionWidget::initializeUsingXML(DOMElement *element) {
    lowerBound->initializeUsingXML(element);
    upperBound->initializeUsingXML(element);
    return element;
  }

  DOMElement* BoundedFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    lowerBound->writeXMLFile(ele0);
    upperBound->writeXMLFile(ele0);
    return ele0;
  }

  VectorValuedFunctionWidget::VectorValuedFunctionWidget(WidgetFactory *factory, int retDim, VarType retType) : FunctionWidget(factory->getElement()) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    factory->setElement(function);

    functions = new ExtWidget("Components",new ListWidget(new ChoiceWidgetFactory(factory,1),"Function",retDim,0,retType==varVec?false:true),false,false,MBSIM%"components");
    layout->addWidget(functions);
  }

  void VectorValuedFunctionWidget::resize_(int m, int n) {
    functions->getWidget<ListWidget>()->setSize(m);
  }

  DOMElement* VectorValuedFunctionWidget::initializeUsingXML(DOMElement *element) {
    functions->initializeUsingXML(element);
    return element;
  }

  DOMElement* VectorValuedFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    functions->writeXMLFile(ele0);
    return ele0;
  }

  CompositeFunctionWidget::CompositeFunctionWidget(WidgetFactory *factoryo1_, WidgetFactory *factoryo2_, WidgetFactory *factoryi_, int defo1, int defo2, int defi) : FunctionWidget(factoryo1_->getElement()), factoryo1(factoryo1_), factoryo2(factoryo2_), factoryi(factoryi_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    factoryo1->setElement(function);
    if(factoryo2) factoryo2->setElement(function);
    factoryi->setElement(function);

    fo = new ExtWidget("Outer function",new ChoiceWidget(factoryo1,QBoxLayout::TopToBottom,0),false,false,MBSIM%"outerFunction");
    layout->addWidget(fo);
    fi = new ChoiceWidget(new CompositeFunctionWidgetFactory(factoryi),QBoxLayout::TopToBottom,5);
    layout->addWidget(fi);
    connect(fo->getWidget<ChoiceWidget>(),&ChoiceWidget::widgetChanged,this,&CompositeFunctionWidget::updateWidget);
    connect(fi,&ChoiceWidget::widgetChanged,this,&CompositeFunctionWidget::updateWidget);
    connect(fo->getWidget<ChoiceWidget>(),&ChoiceWidget::widgetChanged,this,&CompositeFunctionWidget::widgetChanged);
    connect(fi,&ChoiceWidget::widgetChanged,this,&CompositeFunctionWidget::widgetChanged);
    if(factoryo2) connect(fi,&ChoiceWidget::comboChanged,this,&CompositeFunctionWidget::updateFunctionFactory);
  }

  void CompositeFunctionWidget::updateWidget() {
    int size = fo->getFirstWidget<FunctionWidget>()->getArg1Size();
    fi->getWidget<FunctionWidget>()->resize_(size,1);
  }

  void CompositeFunctionWidget::updateFunctionFactory() {
    if(fi->getIndex()==0)
      fo->getWidget<ChoiceWidget>()->setWidgetFactory(factoryo1);
    else
      fo->getWidget<ChoiceWidget>()->setWidgetFactory(factoryo2);
  }

  int CompositeFunctionWidget::getArg1Size() const {
    if(fi->getIndex()==0)
      return fi->getFirstWidget<FunctionWidget>()->getArg1Size();
    else
      return fi->getFirstWidget<ListWidget>()->getWidget<ChoiceWidget>(0)->getWidget<FunctionWidget>()->getArg1Size();
  }

  void CompositeFunctionWidget::resize_(int m, int n) {
    fo->getWidget<ChoiceWidget>()->resize_(m,n);
    fi->resize_(fo->getFirstWidget<FunctionWidget>()->getArg1Size(),n);
  }

  DOMElement* CompositeFunctionWidget::initializeUsingXML(DOMElement *element) {
    fi->initializeUsingXML(element);
    fo->getWidget<ChoiceWidget>()->blockSignals(true);
    updateFunctionFactory();
    fo->getWidget<ChoiceWidget>()->blockSignals(false);
    fo->initializeUsingXML(element);
    return element;
  }

  DOMElement* CompositeFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    fo->writeXMLFile(ele0);
    fi->writeXMLFile(ele0);
    return ele0;
  }

  PiecewiseDefinedFunctionWidget::PiecewiseDefinedFunctionWidget(WidgetFactory *factory) : FunctionWidget(factory->getElement()) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    factory->setElement(function);

    functions = new ExtWidget("Limited functions",new ListWidget(new ChoiceWidgetFactory(new LimitedFunctionWidgetFactory(factory)),"Function",1,0,false,1),false,false,MBSIM%"limitedFunctions");
    layout->addWidget(functions);

    shiftAbscissa = new ExtWidget("Shift abscissa",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"shiftAbscissa");
    layout->addWidget(shiftAbscissa);

    shiftOrdinate = new ExtWidget("Shift ordinate",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"shiftOrdinate");
    layout->addWidget(shiftOrdinate);

    connect(functions,&Widget::widgetChanged,this,&Widget::widgetChanged);
  }

  void PiecewiseDefinedFunctionWidget::resize_(int m, int n) {
    functions->resize_(m,n);
  }

  DOMElement* PiecewiseDefinedFunctionWidget::initializeUsingXML(DOMElement *element) {
    functions->initializeUsingXML(element);
    shiftAbscissa->initializeUsingXML(element);
    shiftOrdinate->initializeUsingXML(element);
    return element;
  }

  DOMElement* PiecewiseDefinedFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    functions->writeXMLFile(ele0);
    shiftAbscissa->writeXMLFile(ele0);
    shiftOrdinate->writeXMLFile(ele0);
    return ele0;
  }

  LimitedFunctionWidget::LimitedFunctionWidget(WidgetFactory *factory) : FunctionWidget(factory->getElement()) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    function = new ExtWidget("Function",new ChoiceWidget(factory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"function");
    layout->addWidget(function);

    limit = new ExtWidget("Limit",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"limit");
    layout->addWidget(limit);
  }

  void LimitedFunctionWidget::resize_(int m, int n) {
    function->resize_(m,n);
  }

  DOMElement* LimitedFunctionWidget::initializeUsingXML(DOMElement *element) {
    function->initializeUsingXML(element);
    limit->initializeUsingXML(element);
    return element;
  }

  DOMElement* LimitedFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    function->writeXMLFile(ele0);
    limit->writeXMLFile(ele0);
    return ele0;
  }

  SymbolicFunctionWidget::SymbolicFunctionWidget(const QStringList &argName, const vector<int> &argDim, const vector<VarType> &argType, int retDim, VarType retType) {
    auto *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    for(int i=0; i<argName.size(); i++) {
      argname.push_back(new ExtWidget("Name of argument "+QString::number(i+1),new TextWidget(argName[i])));
      layout->addWidget(argname[i],i,0);

      argdim.push_back(new ExtWidget("Dimension of argument "+QString::number(i+1),new SpinBoxWidget((argType[i]==scalar)?0:argDim[i],(argType[i]==scalar)?0:1,100)));
      argdim[i]->setDisabled(argType[i]==fixedVec);
      if(argType[i]!=scalar) {
        layout->addWidget(argdim[i],i,1);
        connect(argdim[i]->getWidget<SpinBoxWidget>(),&SpinBoxWidget::valueChanged,this,&SymbolicFunctionWidget::widgetChanged);
      }
    }
    if(retType==scalar)
      f = new ExtWidget("Function",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,"");
    else if(retType==fixedVec)
      f = new ExtWidget("Function",new ChoiceWidget(new VecWidgetFactory(retDim,vector<QStringList>(3,noUnitUnits()),vector<int>(3,0),false,false,false),QBoxLayout::RightToLeft,5),false,false,"");
    else {
      f = new ExtWidget("Function",new ChoiceWidget(new VecSizeVarWidgetFactory(retDim,1,100,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,0),false,false,false),QBoxLayout::RightToLeft,5),false,false,"");
      connect(f,&ExtWidget::widgetChanged,this,&SymbolicFunctionWidget::widgetChanged);
    }
    layout->addWidget(f,argName.size(),0,1,2);
  }

  int SymbolicFunctionWidget::getArg1Size() const {
    return argdim[0]->getWidget<SpinBoxWidget>()->getValue();
  }

  int SymbolicFunctionWidget::getArg2Size() const {
    return argdim[1]->getWidget<SpinBoxWidget>()->getValue();
  }

  void SymbolicFunctionWidget::setArg1Size(int i) {
    argdim[0]->getWidget<SpinBoxWidget>()->setValue(i);
  }

  void SymbolicFunctionWidget::setArg2Size(int i) {
    argdim[1]->getWidget<SpinBoxWidget>()->setValue(i);
  }

  void SymbolicFunctionWidget::resize_(int m, int n) {
    f->resize_(m,n);
  }

  DOMElement* SymbolicFunctionWidget::initializeUsingXML(DOMElement *element) {
    auto definition=E(element)->getFirstElementChildNamed(MBSIM%"definition");

    f->initializeUsingXML(definition);

    for(size_t i=0; i<argname.size(); i++) {
      string str = "arg"+toStr(int(i+1));
      if(E(definition)->hasAttribute(str))
        argname[i]->getWidget<TextWidget>()->setText(QString::fromStdString(E(definition)->getAttribute(str)));
      str = "arg"+toStr(int(i+1))+"Dim";
      if(E(definition)->hasAttribute(str))
        argdim[i]->getWidget<SpinBoxWidget>()->setValue(boost::lexical_cast<int>(E(definition)->getAttribute(str)));
    }
    return element;
  }

  DOMElement* SymbolicFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);

    DOMElement *definition=D(ele0->getOwnerDocument())->createElement(MBSIM%"definition");
    ele0->insertBefore(definition, ref);

    for(size_t i=0; i<argname.size(); i++) {
      string istr = toStr(int(i+1));
      E(definition)->setAttribute("arg"+istr, argname[i]->getWidget<TextWidget>()->getText().toStdString());
      if(argdim[i]->getWidget<SpinBoxWidget>()->getValue()!=0)
        E(definition)->setAttribute("arg"+istr+"Dim",fmatvec::toString(argdim[i]->getWidget<SpinBoxWidget>()->getValue()));
    }
    f->writeXMLFile(definition);
    return ele0;
  }

  TabularFunctionWidget::TabularFunctionWidget(int retDim, VarType retType) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget(new TabularFunctionWidgetFactory(retDim,retType),QBoxLayout::TopToBottom,5);
    layout->addWidget(choice);

    choiceChanged();
    connect(choice,&ChoiceWidget::widgetChanged,this,&TabularFunctionWidget::choiceChanged);
    connect(choice,&ChoiceWidget::widgetChanged,this,&TabularFunctionWidget::widgetChanged);
  }

  void TabularFunctionWidget::choiceChanged() {
    if(choice->getIndex()==0) {
      updateWidget();
      connect(choice->getWidget<ContainerWidget>()->getWidget<Widget>(0),&Widget::widgetChanged,this,&TabularFunctionWidget::updateWidget);
      connect(choice->getWidget<ContainerWidget>()->getWidget<Widget>(0),&Widget::widgetChanged,this,&TabularFunctionWidget::updateWidget);
    }
  }

  void TabularFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      auto *choice1_ = choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(0)->getWidget<ChoiceWidget>();
      auto *choice2_ = choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(1)->getWidget<ChoiceWidget>();
      choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(1)->resize_(choice1_->getWidget<PhysicalVariableWidget>()->rows(),choice2_->getFirstWidget<VariableWidget>()->cols());
    }
  }

  void TabularFunctionWidget::resize_(int m, int n) {
    if(choice->getIndex()==0) {
      auto *choice_ = choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(0)->getWidget<ChoiceWidget>();
      choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(1)->resize_(choice_->getWidget<PhysicalVariableWidget>()->rows(),m);
    }
    else {
      auto *choice_ = choice->getFirstWidget<ChoiceWidget>();
      choice->resize_(choice_->getWidget<PhysicalVariableWidget>()->rows(),m+1);
    }
  }

  DOMElement* TabularFunctionWidget::initializeUsingXML(DOMElement *element) {
    choice->initializeUsingXML(element);
    return element;
  }

  DOMElement* TabularFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    choice->writeXMLFile(ele0);
    return ele0;
  }

  TwoDimensionalTabularFunctionWidget::TwoDimensionalTabularFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget(new TwoDimensionalTabularFunctionWidgetFactory,QBoxLayout::TopToBottom,5);
    layout->addWidget(choice);

    choiceChanged();
    connect(choice,&ChoiceWidget::widgetChanged,this,&TwoDimensionalTabularFunctionWidget::choiceChanged);
  }

  void TwoDimensionalTabularFunctionWidget::choiceChanged() {
    if(choice->getIndex()==0) {
      connect(choice->getWidget<ContainerWidget>()->getWidget<Widget>(0),&Widget::widgetChanged,this,&TwoDimensionalTabularFunctionWidget::updateWidget);
      connect(choice->getWidget<ContainerWidget>()->getWidget<Widget>(1),&Widget::widgetChanged,this,&TwoDimensionalTabularFunctionWidget::updateWidget);
    }
  }

  void TwoDimensionalTabularFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      auto *choice1_ = choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(0)->getWidget<ChoiceWidget>();
      auto *choice2_ = choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(1)->getWidget<ChoiceWidget>();
      choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(2)->resize_(choice2_->getWidget<PhysicalVariableWidget>()->rows(),choice1_->getFirstWidget<VariableWidget>()->rows());
    }
  }

  DOMElement* TwoDimensionalTabularFunctionWidget::initializeUsingXML(DOMElement *element) {
    choice->initializeUsingXML(element);
    return element;
  }

  DOMElement* TwoDimensionalTabularFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    choice->writeXMLFile(ele0);
    return ele0;
  }

  PiecewisePolynomFunctionWidget::PiecewisePolynomFunctionWidget(int retDim, VarType retType) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choiceXY = new ChoiceWidget(new TabularFunctionWidgetFactory(retDim,retType),QBoxLayout::TopToBottom,5);
    layout->addWidget(choiceXY);

    vector<QString> list;
    list.emplace_back("\"cSplinePeriodic\"");
    list.emplace_back("\"cSplineNatural\"");
    list.emplace_back("\"piecewiseLinear\"");
    interpolationMethod = new ExtWidget("Interpolation method",new TextChoiceWidget(list,1,true),true,false,MBSIM%"interpolationMethod");
    layout->addWidget(interpolationMethod);

    vector<QString> list2;
    list2.emplace_back("\"error\"");
    list2.emplace_back("\"continuePolynom\"");
    list2.emplace_back("\"linear\"");
    extrapolationMethod = new ExtWidget("Extrapolation method",new TextChoiceWidget(list2,1,true),true,false,MBSIM%"extrapolationMethod");
    layout->addWidget(extrapolationMethod);

    choiceChanged();
    connect(choiceXY,&ChoiceWidget::widgetChanged,this,&PiecewisePolynomFunctionWidget::choiceChanged);
    connect(choiceXY,&ChoiceWidget::widgetChanged,this,&PiecewisePolynomFunctionWidget::widgetChanged);
  }

  void PiecewisePolynomFunctionWidget::choiceChanged() {
    if(fallbackWidget)
      return;

    if(choiceXY->getIndex()==0) {
      updateWidget();
      connect(choiceXY->getWidget<ContainerWidget>()->getWidget<Widget>(0),&Widget::widgetChanged,this,&PiecewisePolynomFunctionWidget::updateWidget);
    }
  }

  void PiecewisePolynomFunctionWidget::updateWidget() {
    if(fallbackWidget)
      return;

    if(choiceXY->getIndex()==0) {
      auto *choice1_ = choiceXY->getWidget<ContainerWidget>()->getWidget<ExtWidget>(0)->getWidget<ChoiceWidget>();
      auto *choice2_ = choiceXY->getWidget<ContainerWidget>()->getWidget<ExtWidget>(1)->getWidget<ChoiceWidget>();
      choiceXY->getWidget<ContainerWidget>()->getWidget<ExtWidget>(1)->resize_(choice1_->getWidget<PhysicalVariableWidget>()->rows(),choice2_->getFirstWidget<VariableWidget>()->cols());
    }
  }

  void PiecewisePolynomFunctionWidget::resize_(int m, int n) {
    if(fallbackWidget)
      return;

    if(choiceXY->getIndex()==0) {
      auto *choice_ = choiceXY->getWidget<ContainerWidget>()->getWidget<ExtWidget>(0)->getWidget<ChoiceWidget>();
      choiceXY->getWidget<ContainerWidget>()->getWidget<ExtWidget>(1)->resize_(choice_->getWidget<PhysicalVariableWidget>()->rows(),m);
    }
    else {
      auto *choice_ = choiceXY->getFirstWidget<ChoiceWidget>();
      choiceXY->resize_(choice_->getWidget<PhysicalVariableWidget>()->rows(),m+1);
    }
  }

  DOMElement* PiecewisePolynomFunctionWidget::initializeUsingXML(DOMElement *element) {
    if(E(element)->getFirstElementChildNamed(MBSIM%"breaks")) {
      fallbackWidget = new XMLEditorWidget;
      choiceXY->setVisible(false);
      interpolationMethod->setVisible(false);
      extrapolationMethod->setVisible(false);
      layout()->addWidget(fallbackWidget);

      fallbackWidget->initializeUsingXML(element);
      return element;
    }

    choiceXY->initializeUsingXML(element);
    interpolationMethod->initializeUsingXML(element);
    return element;
  }

  DOMElement* PiecewisePolynomFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    if(fallbackWidget) {
      DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
      fallbackWidget->writeXMLFile(ele0);
      return ele0;
    }

    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    choiceXY->writeXMLFile(ele0);
    interpolationMethod->writeXMLFile(ele0);
    return ele0;
  }

  TwoDimensionalPiecewisePolynomFunctionWidget::TwoDimensionalPiecewisePolynomFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget(new TwoDimensionalTabularFunctionWidgetFactory,QBoxLayout::TopToBottom,3);
    layout->addWidget(choice);

    vector<QString> list;
    list.emplace_back("\"cSplinePeriodic\"");
    list.emplace_back("\"cSplineNatural\"");
    list.emplace_back("\"piecewiseLinear\"");
    method = new ExtWidget("Interpolation method",new TextChoiceWidget(list,1,true),true);
    layout->addWidget(method);

    choiceChanged();
    connect(choice,&ChoiceWidget::widgetChanged,this,&TwoDimensionalPiecewisePolynomFunctionWidget::choiceChanged);
  }

  void TwoDimensionalPiecewisePolynomFunctionWidget::choiceChanged() {
    if(choice->getIndex()==0) {
      connect(choice->getWidget<ContainerWidget>()->getWidget<Widget>(0),&Widget::widgetChanged,this,&TwoDimensionalPiecewisePolynomFunctionWidget::updateWidget);
      connect(choice->getWidget<ContainerWidget>()->getWidget<Widget>(1),&Widget::widgetChanged,this,&TwoDimensionalPiecewisePolynomFunctionWidget::updateWidget);
    }
  }

  void TwoDimensionalPiecewisePolynomFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      auto *choice1_ = choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(0)->getWidget<ChoiceWidget>();
      auto *choice2_ = choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(1)->getWidget<ChoiceWidget>();
      choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(2)->resize_(choice2_->getWidget<PhysicalVariableWidget>()->rows(),choice1_->getFirstWidget<VariableWidget>()->rows());
    }
  }

  DOMElement* TwoDimensionalPiecewisePolynomFunctionWidget::initializeUsingXML(DOMElement *element) {
    choice->initializeUsingXML(element);
    return element;
  }

  DOMElement* TwoDimensionalPiecewisePolynomFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    choice->writeXMLFile(ele0);
    return ele0;
  }

  FourierFunctionWidget::FourierFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    f = new ExtWidget("Frequency",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frequency");
    layout->addWidget(f);

    a0 = new ExtWidget("a0",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a0");
    layout->addWidget(a0);

    choice = new ChoiceWidget(new FourierFunctionWidgetFactory,QBoxLayout::TopToBottom,5);
    layout->addWidget(choice);

    amplitudePhaseForm = new ExtWidget("Amplitude phase form",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"amplitudePhaseForm");
    layout->addWidget(amplitudePhaseForm);
  }

  void FourierFunctionWidget::resize_(int m, int n) {
    if(choice->getIndex()==0) {
      auto *choice_ = choice->getWidget<ContainerWidget>()->getWidget<ExtWidget>(0)->getWidget<ChoiceWidget>();
      if(choice_->getIndex()==0)
        choice->resize_(choice_->getFirstWidget<VecSizeVarWidget>()->size(),m);
    }
    else {
      auto *choice_ = choice->getFirstWidget<ChoiceWidget>();
      if(choice_->getIndex()==0)
        choice->resize_(choice_->getFirstWidget<MatRowsVarWidget>()->rows(),m+1);
    }
  }

  DOMElement* FourierFunctionWidget::initializeUsingXML(DOMElement *element) {
    f->initializeUsingXML(element);
    a0->initializeUsingXML(element);
    choice->initializeUsingXML(element);
    amplitudePhaseForm->initializeUsingXML(element);
    return element;
  }

  DOMElement* FourierFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    f->writeXMLFile(ele0);
    a0->writeXMLFile(ele0);
    choice->writeXMLFile(ele0);
    amplitudePhaseForm->writeXMLFile(ele0);
    return ele0;
  }

  BidirectionalFunctionWidget::BidirectionalFunctionWidget(Element *parentElement, const QString &argName, int argDim, VarType argType, int retDim, VarType retType, QWidget *parent) : FunctionWidget(parentElement) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    fn = new ExtWidget("Negative directional function",new ChoiceWidget(new Function1ArgWidgetFactory(function,argName,argDim,argType,retDim,retType,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"negativeDirectionalFunction");
    layout->addWidget(fn);
    fp = new ExtWidget("Positive directional function",new ChoiceWidget(new Function1ArgWidgetFactory(function,argName,argDim,argType,retDim,retType,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"positiveDirectionalFunction");
    layout->addWidget(fp);
  }

  void BidirectionalFunctionWidget::resize_(int m, int n) {
    fn->getWidget<ChoiceWidget>()->resize_(m,n);
    fp->getWidget<ChoiceWidget>()->resize_(m,n);
  }

  DOMElement* BidirectionalFunctionWidget::initializeUsingXML(DOMElement *element) {
    fn->initializeUsingXML(element);
    fp->initializeUsingXML(element);
    return element;
  }

  DOMElement* BidirectionalFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    fn->writeXMLFile(ele0);
    fp->writeXMLFile(ele0);
    return ele0;
  }

  ContinuedFunctionWidget::ContinuedFunctionWidget(WidgetFactory *factoryf, WidgetFactory *factoryr) : FunctionWidget(factoryf->getElement()) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    factoryf->setElement(function);
    factoryr->setElement(function);

    f = new ExtWidget("Function",new ChoiceWidget(factoryf,QBoxLayout::TopToBottom,0),false,false,MBSIM%"function");
    layout->addWidget(f);

    r = new ExtWidget("Continuation rule",new ChoiceWidget(factoryr,QBoxLayout::TopToBottom,0),false,false,MBSIM%"continuationRule");
    layout->addWidget(r);
  }

  void ContinuedFunctionWidget::resize_(int m, int n) {
    f->getWidget<ChoiceWidget>()->resize_(m,n);
    r->getWidget<ChoiceWidget>()->resize_(n,n);
  }

  DOMElement* ContinuedFunctionWidget::initializeUsingXML(DOMElement *element) {
    f->initializeUsingXML(element);
    r->initializeUsingXML(element);
    return element;
  }

  DOMElement* ContinuedFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    f->writeXMLFile(ele0);
    r->writeXMLFile(ele0);
    return ele0;
  }

  LinearSpringDamperForceWidget::LinearSpringDamperForceWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    c = new ExtWidget("Stiffness coefficient",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessCoefficient");
    layout->addWidget(c);

    d = new ExtWidget("Damping coefficient",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dampingCoefficient");
    layout->addWidget(d);
  }

  DOMElement* LinearSpringDamperForceWidget::initializeUsingXML(DOMElement *element) {
    c->initializeUsingXML(element);
    d->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearSpringDamperForceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    c->writeXMLFile(ele0);
    d->writeXMLFile(ele0);
    return ele0;
  }

  NonlinearSpringDamperForceWidget::NonlinearSpringDamperForceWidget(Element *parentElement, QWidget *parent) : FunctionWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    s = new ExtWidget("Force deflection function",new ChoiceWidget(new Function1ArgWidgetFactory(function,"s",1,scalar,1,scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceDeflectionFunction");
    layout->addWidget(s);

    sd = new ExtWidget("Force velocity function",new ChoiceWidget(new Function1ArgWidgetFactory(function,"sd",1,scalar,1,scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceVelocityFunction");
    layout->addWidget(sd);
  }

  DOMElement* NonlinearSpringDamperForceWidget::initializeUsingXML(DOMElement *element) {
    s->initializeUsingXML(element);
    sd->initializeUsingXML(element);
    return element;
  }

  DOMElement* NonlinearSpringDamperForceWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    s->writeXMLFile(ele0);
    sd->writeXMLFile(ele0);
    return ele0;
  }

  LinearElasticFunctionWidget::LinearElasticFunctionWidget(bool varSize) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    K = varSize?new ExtWidget("Generalized stiffness matrix",new ChoiceWidget(new SymMatSizeVarWidgetFactory(getEye<QString>(3,3,"0","0"),vector<QStringList>(3),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessMatrix"):new ExtWidget("Generalized stiffness matrix",new ChoiceWidget(new SymMatWidgetFactory(getEye<QString>(3,3,"0","0"),vector<QStringList>(3),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessMatrix");
    layout->addWidget(K);
    D = new ExtWidget("Generalized damping matrix",new ChoiceWidget(new SymMatWidgetFactory(getEye<QString>(3,3,"0","0"),vector<QStringList>(3),vector<int>(3,2)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"dampingMatrix");
    layout->addWidget(D);

    connect(K->getWidget<ChoiceWidget>(),&ChoiceWidget::widgetChanged,this,&LinearElasticFunctionWidget::updateWidget);
    connect(D,&ExtWidget::widgetChanged,this,&LinearElasticFunctionWidget::updateWidget);
  }

  void LinearElasticFunctionWidget::updateWidget() {
    if(D->isActive()) {
      int size = K->getFirstWidget<PhysicalVariableWidget>()->rows();
      D->resize_(size,size);
    }
  }

  void LinearElasticFunctionWidget::resize_(int m, int n) {
    K->resize_(m,n);
    if(D->isActive()) D->resize_(m,n);
  }

  DOMElement* LinearElasticFunctionWidget::initializeUsingXML(DOMElement *element) {
    K->initializeUsingXML(element);
    D->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearElasticFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    K->writeXMLFile(ele0);
    D->writeXMLFile(ele0);
    return ele0;
  }

  LinearRegularizedBilateralConstraintWidget::LinearRegularizedBilateralConstraintWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    c = new ExtWidget("Stiffness coefficient",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessCoefficient");
    layout->addWidget(c);

    d = new ExtWidget("Damping coefficient",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dampingCoefficient");
    layout->addWidget(d);
  }

  DOMElement* LinearRegularizedBilateralConstraintWidget::initializeUsingXML(DOMElement *element) {
    c->initializeUsingXML(element);
    d->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedBilateralConstraintWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    c->writeXMLFile(ele0);
    d->writeXMLFile(ele0);
    return ele0;
  }

  LinearRegularizedUnilateralConstraintWidget::LinearRegularizedUnilateralConstraintWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    c = new ExtWidget("Stiffness coefficient",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessCoefficient");
    layout->addWidget(c);

    d = new ExtWidget("Damping coefficient",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dampingCoefficient");
    layout->addWidget(d);
  }

  DOMElement* LinearRegularizedUnilateralConstraintWidget::initializeUsingXML(DOMElement *element) {
    c->initializeUsingXML(element);
    d->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedUnilateralConstraintWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    c->writeXMLFile(ele0);
    d->writeXMLFile(ele0);
    return ele0;
  }

  LinearRegularizedCoulombFrictionWidget::LinearRegularizedCoulombFrictionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    gd = new ExtWidget("Marginal velocity",new ChoiceWidget(new ScalarWidgetFactory("0.01",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"marginalVelocity");
    layout->addWidget(gd);

    mu = new ExtWidget("Friction coefficient",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
    layout->addWidget(mu);
  }

  DOMElement* LinearRegularizedCoulombFrictionWidget::initializeUsingXML(DOMElement *element) {
    gd->initializeUsingXML(element);
    mu->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedCoulombFrictionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    gd->writeXMLFile(ele0);
    mu->writeXMLFile(ele0);
    return ele0;
  }

  LinearRegularizedStribeckFrictionWidget::LinearRegularizedStribeckFrictionWidget(Element *parentElement, QWidget *parent) : FunctionWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    gd = new ExtWidget("Marginal velocity",new ChoiceWidget(new ScalarWidgetFactory("0.01",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"marginalVelocity");
    layout->addWidget(gd);

    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(function,"v",1,FunctionWidget::scalar,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
    layout->addWidget(frictionFunction);
  }

  DOMElement* LinearRegularizedStribeckFrictionWidget::initializeUsingXML(DOMElement *element) {
    gd->initializeUsingXML(element);
    frictionFunction->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedStribeckFrictionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    gd->writeXMLFile(ele0);
    frictionFunction->writeXMLFile(ele0);
    return ele0;
  }

  SignalFunctionWidget::SignalFunctionWidget(Element *parentElement, QWidget *parent) : FunctionWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    sRef = new ExtWidget("Return signal",new ElementOfReferenceWidget<Signal>(function,nullptr,parent),false,false,MBSIMCONTROL%"returnSignal");
    layout->addWidget(sRef);
  }
  
  void SignalFunctionWidget::updateWidget() { 
    sRef->updateWidget(); 
  }

  DOMElement* SignalFunctionWidget::initializeUsingXML(DOMElement *element) {
    sRef->initializeUsingXML(element);
    return element;
  }

  DOMElement* SignalFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    sRef->writeXMLFile(ele0);
    return ele0;
  }

  PolarContourFunctionWidget::PolarContourFunctionWidget(Element *parentElement, QWidget *parent) : FunctionWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    radiusFunction = new ExtWidget("Radius function",new ChoiceWidget(new Function1ArgWidgetFactory(function,"phi",1,scalar,1,scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"radiusFunction");
    layout->addWidget(radiusFunction);
  }

  DOMElement* PolarContourFunctionWidget::initializeUsingXML(DOMElement *element) {
    radiusFunction->initializeUsingXML(element);
    return element;
  }

  DOMElement* PolarContourFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    radiusFunction->writeXMLFile(ele0);
    return ele0;
  }

  GravityFunctionWidget::GravityFunctionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    standardGravity = new ExtWidget("Standard gravity",new ChoiceWidget(new ScalarWidgetFactory("9.80665",vector<QStringList>(2,accelerationUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIMPHYSICS%"standardGravity");
    layout->addWidget(standardGravity);

    meanRadius = new ExtWidget("Mean radius",new ChoiceWidget(new ScalarWidgetFactory("6371e3",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIMPHYSICS%"meanRadius");
    layout->addWidget(meanRadius);
  }

  DOMElement* GravityFunctionWidget::initializeUsingXML(DOMElement *element) {
    standardGravity->initializeUsingXML(element);
    meanRadius->initializeUsingXML(element);
    return element;
  }

  DOMElement* GravityFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    standardGravity->writeXMLFile(ele0);
    meanRadius->writeXMLFile(ele0);
    return ele0;
  }

}
