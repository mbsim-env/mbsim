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

  VectorValuedFunctionWidget::VectorValuedFunctionWidget(WidgetFactory *factory, int retDim, VarType retType) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    functions = new ExtWidget("Components",new ListWidget(new ChoiceWidgetFactory(factory,1),"Function",retDim,0,retType),false,false,MBSIM%"components");
    layout->addWidget(functions);
  }

  void VectorValuedFunctionWidget::resize_(int m, int n) {
    static_cast<ListWidget*>(functions->getWidget())->setSize(m);
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

  CompositeFunctionWidget::CompositeFunctionWidget(WidgetFactory *factoryo1_, WidgetFactory *factoryo2_, WidgetFactory *factoryi_, int defo1, int defo2, int defi) : factoryo1(factoryo1_), factoryo2(factoryo2_), factoryi(factoryi_) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    fo = new ExtWidget("Outer function",new ChoiceWidget(factoryo1,QBoxLayout::TopToBottom,0),false,false,MBSIM%"outerFunction");
    layout->addWidget(fo);
    fi = new ChoiceWidget(new CompositeFunctionWidgetFactory(factoryi),QBoxLayout::TopToBottom,5);
    layout->addWidget(fi);
    connect(static_cast<ChoiceWidget*>(fo->getWidget()),&ChoiceWidget::widgetChanged,this,&CompositeFunctionWidget::updateWidget);
    connect(fi,&ChoiceWidget::widgetChanged,this,&CompositeFunctionWidget::updateWidget);
    connect(static_cast<ChoiceWidget*>(fo->getWidget()),&ChoiceWidget::widgetChanged,this,&CompositeFunctionWidget::widgetChanged);
    connect(fi,&ChoiceWidget::widgetChanged,this,&CompositeFunctionWidget::widgetChanged);
    if(factoryo2) connect(fi,&ChoiceWidget::comboChanged,this,&CompositeFunctionWidget::updateFunctionFactory);
  }

  void CompositeFunctionWidget::updateWidget() {
    int size = static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(fo->getWidget())->getWidget())->getArg1Size();
    static_cast<FunctionWidget*>(fi->getWidget())->resize_(size,1);
  }

  void CompositeFunctionWidget::updateFunctionFactory() {
    if(fi->getIndex()==0)
      static_cast<ChoiceWidget*>(fo->getWidget())->setWidgetFactory(factoryo1);
    else
      static_cast<ChoiceWidget*>(fo->getWidget())->setWidgetFactory(factoryo2);
  }

  int CompositeFunctionWidget::getArg1Size() const {
    if(fi->getIndex()==0)
      return static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(fi->getWidget())->getWidget())->getWidget())->getArg1Size();
    else
      return static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(static_cast<ListWidget*>(static_cast<ExtWidget*>(fi->getWidget())->getWidget())->getWidget(0))->getWidget())->getArg1Size();
  }

  void CompositeFunctionWidget::resize_(int m, int n) {
    static_cast<ChoiceWidget*>(fo->getWidget())->resize_(m,n);
    static_cast<ChoiceWidget*>(fi)->resize_(static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(fo->getWidget())->getWidget())->getArg1Size(),n);
  }

  DOMElement* CompositeFunctionWidget::initializeUsingXML(DOMElement *element) {
    fi->initializeUsingXML(element);
    updateFunctionFactory();
    fo->initializeUsingXML(element);
    return element;
  }

  DOMElement* CompositeFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    fo->writeXMLFile(ele0);
    fi->writeXMLFile(ele0);
    return ele0;
  }

  PiecewiseDefinedFunctionWidget::PiecewiseDefinedFunctionWidget(WidgetFactory *factory) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    functions = new ExtWidget("Limited functions",new ListWidget(new ChoiceWidgetFactory(new LimitedFunctionWidgetFactory(factory)),"Function",1,0,false,1),false,false,MBSIM%"limitedFunctions");
    layout->addWidget(functions);

    shiftAbscissa = new ExtWidget("Shift abscissa",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"shiftAbscissa");
    layout->addWidget(shiftAbscissa);

    shiftOrdinate = new ExtWidget("Shift ordinate",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"shiftOrdinate");
    layout->addWidget(shiftOrdinate);
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

  LimitedFunctionWidget::LimitedFunctionWidget(WidgetFactory *factory) {
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
        connect(static_cast<SpinBoxWidget*>(argdim[i]->getWidget()),&SpinBoxWidget::valueChanged,this,&SymbolicFunctionWidget::widgetChanged);
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
    return static_cast<SpinBoxWidget*>(argdim[0]->getWidget())->getValue();
  }

  int SymbolicFunctionWidget::getArg2Size() const {
    return static_cast<SpinBoxWidget*>(argdim[1]->getWidget())->getValue();
  }

  void SymbolicFunctionWidget::setArg1Size(int i) {
    static_cast<SpinBoxWidget*>(argdim[0]->getWidget())->setValue(i);
  }

  void SymbolicFunctionWidget::setArg2Size(int i) {
    static_cast<SpinBoxWidget*>(argdim[1]->getWidget())->setValue(i);
  }

  void SymbolicFunctionWidget::resize_(int m, int n) {
    f->resize_(m,n);
  }

  DOMElement* SymbolicFunctionWidget::initializeUsingXML(DOMElement *element) {
    f->initializeUsingXML(element);
    for(size_t i=0; i<argname.size(); i++) {
      string str = "arg"+toStr(int(i+1));
      if(E(element)->hasAttribute(str))
        static_cast<TextWidget*>(argname[i]->getWidget())->setText(QString::fromStdString(E(element)->getAttribute(str)));
      str = "arg"+toStr(int(i+1))+"Dim";
      if(E(element)->hasAttribute(str))
        static_cast<SpinBoxWidget*>(argdim[i]->getWidget())->setValue(boost::lexical_cast<int>(E(element)->getAttribute(str)));
    }
    return element;
  }

  DOMElement* SymbolicFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    for(size_t i=0; i<argname.size(); i++) {
      string istr = toStr(int(i+1));
      E(ele0)->setAttribute("arg"+istr, static_cast<TextWidget*>(argname[i]->getWidget())->getText().toStdString());
      if(static_cast<SpinBoxWidget*>(argdim[i]->getWidget())->getValue()!=0)
        E(ele0)->setAttribute("arg"+istr+"Dim",fmatvec::toString(static_cast<SpinBoxWidget*>(argdim[i]->getWidget())->getValue()));
    }
    f->writeXMLFile(ele0);
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
      connect(static_cast<Widget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0)),&Widget::widgetChanged,this,&TabularFunctionWidget::updateWidget);
      connect(static_cast<Widget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0)),&Widget::widgetChanged,this,&TabularFunctionWidget::updateWidget);
    }
  }

  void TabularFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      auto *choice1_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      auto *choice2_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->resize_(static_cast<PhysicalVariableWidget*>(choice1_->getWidget())->rows(),static_cast<PhysicalVariableWidget*>(choice2_->getWidget())->getWidget()->cols());
    }
  }

  void TabularFunctionWidget::resize_(int m, int n) {
    if(choice->getIndex()==0) {
      auto *choice_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->resize_(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->rows(),m);
    }
    else {
      auto *choice_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(choice->getWidget())->getWidget());
      choice->resize_(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->rows(),m+1);
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
      connect(static_cast<Widget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0)),&Widget::widgetChanged,this,&TwoDimensionalTabularFunctionWidget::updateWidget);
      connect(static_cast<Widget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1)),&Widget::widgetChanged,this,&TwoDimensionalTabularFunctionWidget::updateWidget);
    }
  }

  void TwoDimensionalTabularFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      auto *choice1_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      auto *choice2_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(2))->resize_(static_cast<PhysicalVariableWidget*>(choice2_->getWidget())->rows(),static_cast<PhysicalVariableWidget*>(choice1_->getWidget())->getWidget()->rows());
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

    choice = new ChoiceWidget(new TabularFunctionWidgetFactory(retDim,retType),QBoxLayout::TopToBottom,5);
    layout->addWidget(choice);

    vector<QString> list;
    list.emplace_back("\"cSplinePeriodic\"");
    list.emplace_back("\"cSplineNatural\"");
    list.emplace_back("\"piecewiseLinear\"");
    method = new ExtWidget("Interpolation method",new TextChoiceWidget(list,1,true),true,false,MBSIM%"interpolationMethod");
    layout->addWidget(method);

    choiceChanged();
    connect(choice,&ChoiceWidget::widgetChanged,this,&PiecewisePolynomFunctionWidget::choiceChanged);
    connect(choice,&ChoiceWidget::widgetChanged,this,&PiecewisePolynomFunctionWidget::widgetChanged);
  }

  void PiecewisePolynomFunctionWidget::choiceChanged() {
    if(choice->getIndex()==0) {
      updateWidget();
      connect(static_cast<Widget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0)),&Widget::widgetChanged,this,&PiecewisePolynomFunctionWidget::updateWidget);
    }
  }

  void PiecewisePolynomFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      auto *choice1_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      auto *choice2_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->resize_(static_cast<PhysicalVariableWidget*>(choice1_->getWidget())->rows(),static_cast<PhysicalVariableWidget*>(choice2_->getWidget())->getWidget()->cols());
    }
  }

  void PiecewisePolynomFunctionWidget::resize_(int m, int n) {
    if(choice->getIndex()==0) {
      auto *choice_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->resize_(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->rows(),m);
    }
    else {
      auto *choice_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(choice->getWidget())->getWidget());
      choice->resize_(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->rows(),m+1);
    }
  }

  DOMElement* PiecewisePolynomFunctionWidget::initializeUsingXML(DOMElement *element) {
    choice->initializeUsingXML(element);
    method->initializeUsingXML(element);
    return element;
  }

  DOMElement* PiecewisePolynomFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    choice->writeXMLFile(ele0);
    method->writeXMLFile(ele0);
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
      connect(static_cast<Widget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0)),&Widget::widgetChanged,this,&TwoDimensionalPiecewisePolynomFunctionWidget::updateWidget);
      connect(static_cast<Widget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1)),&Widget::widgetChanged,this,&TwoDimensionalPiecewisePolynomFunctionWidget::updateWidget);
    }
  }

  void TwoDimensionalPiecewisePolynomFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      auto *choice1_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      auto *choice2_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(2))->resize_(static_cast<PhysicalVariableWidget*>(choice2_->getWidget())->rows(),static_cast<PhysicalVariableWidget*>(choice1_->getWidget())->getWidget()->rows());
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

    f = new ExtWidget("Frequency",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"frequency");
    layout->addWidget(f);

    a0 = new ExtWidget("a0",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a0");
    layout->addWidget(a0);

    choice = new ChoiceWidget(new FourierFunctionWidgetFactory,QBoxLayout::TopToBottom,5);
    layout->addWidget(choice);

    amplitudePhaseAngleForm = new ExtWidget("Amplitude phase angle form",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"amplitudePhaseAngleForm");
    layout->addWidget(amplitudePhaseAngleForm);
  }

  void FourierFunctionWidget::resize_(int m, int n) {
    if(choice->getIndex()==0) {
      auto *choice_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      if(choice_->getIndex()==0)
        choice->resize_(static_cast<VecSizeVarWidget*>(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->getWidget())->size(),m);
    }
    else {
      auto *choice_ = static_cast<ChoiceWidget*>(static_cast<ExtWidget*>(choice->getWidget())->getWidget());
      if(choice_->getIndex()==0)
        choice->resize_(static_cast<MatRowsVarWidget*>(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->getWidget())->rows(),m+1);
    }
  }

  DOMElement* FourierFunctionWidget::initializeUsingXML(DOMElement *element) {
    f->initializeUsingXML(element);
    a0->initializeUsingXML(element);
    choice->initializeUsingXML(element);
    amplitudePhaseAngleForm->initializeUsingXML(element);
    return element;
  }

  DOMElement* FourierFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    f->writeXMLFile(ele0);
    a0->writeXMLFile(ele0);
    choice->writeXMLFile(ele0);
    amplitudePhaseAngleForm->writeXMLFile(ele0);
    return ele0;
  }

  BidirectionalFunctionWidget::BidirectionalFunctionWidget(Element *element, const QString &argName, int argDim, VarType argType, int retDim, VarType retType, QWidget *parent) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    fn = new ExtWidget("Negative directional function",new ChoiceWidget(new Function1ArgWidgetFactory(element,argName,argDim,argType,retDim,retType,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"negativeDirectionalFunction");
    layout->addWidget(fn);
    fp = new ExtWidget("Positive directional function",new ChoiceWidget(new Function1ArgWidgetFactory(element,argName,argDim,argType,retDim,retType,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"positiveDirectionalFunction");
    layout->addWidget(fp);
  }

  void BidirectionalFunctionWidget::resize_(int m, int n) {
    static_cast<ChoiceWidget*>(fn->getWidget())->resize_(m,n);
    static_cast<ChoiceWidget*>(fp->getWidget())->resize_(m,n);
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

  ContinuedFunctionWidget::ContinuedFunctionWidget(WidgetFactory *factoryf, WidgetFactory *factoryr) {

    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    f = new ExtWidget("Function",new ChoiceWidget(factoryf,QBoxLayout::TopToBottom,0),false,false,MBSIM%"function");
    layout->addWidget(f);

    r = new ExtWidget("Continuation rule",new ChoiceWidget(factoryr,QBoxLayout::TopToBottom,0),false,false,MBSIM%"continuationRule");
    layout->addWidget(r);
  }

  void ContinuedFunctionWidget::resize_(int m, int n) {
    static_cast<ChoiceWidget*>(f->getWidget())->resize_(m,n);
    static_cast<ChoiceWidget*>(r->getWidget())->resize_(n,n);
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

  NonlinearSpringDamperForceWidget::NonlinearSpringDamperForceWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    s = new ExtWidget("Force deflection function",new ChoiceWidget(new Function1ArgWidgetFactory(element,"s",1,scalar,1,scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceDeflectionFunction");
    layout->addWidget(s);

    sd = new ExtWidget("Force velocity function",new ChoiceWidget(new Function1ArgWidgetFactory(element,"sd",1,scalar,1,scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceVelocityFunction");
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

    connect(static_cast<ChoiceWidget*>(K->getWidget()),&ChoiceWidget::widgetChanged,this,&LinearElasticFunctionWidget::updateWidget);
    connect(D,&ExtWidget::widgetChanged,this,&LinearElasticFunctionWidget::updateWidget);
  }

  void LinearElasticFunctionWidget::updateWidget() {
    if(D->isActive()) {
      int size = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget*>(K->getWidget())->getWidget())->rows();
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

  LinearRegularizedStribeckFrictionWidget::LinearRegularizedStribeckFrictionWidget() {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    gd = new ExtWidget("Marginal velocity",new ChoiceWidget(new ScalarWidgetFactory("0.01",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"marginalVelocity");
    layout->addWidget(gd);

    mu = new ExtWidget("Friction coefficient",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
    layout->addWidget(mu);
  }

  DOMElement* LinearRegularizedStribeckFrictionWidget::initializeUsingXML(DOMElement *element) {
    gd->initializeUsingXML(element);
    mu->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearRegularizedStribeckFrictionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    gd->writeXMLFile(ele0);
    mu->writeXMLFile(ele0);
    return ele0;
  }

  SignalFunctionWidget::SignalFunctionWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    dummy = new Function; // Workaround for correct XML path. TODO: provide a consistent concept
    dummy->setParent(element);
    sRef = new ExtWidget("Return signal",new ElementOfReferenceWidget<Signal>(dummy,nullptr,parent),false,false,MBSIMCONTROL%"returnSignal");
    layout->addWidget(sRef);
  }
  
  SignalFunctionWidget::~SignalFunctionWidget() { 
    delete dummy; 
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

  PolarContourFunctionWidget::PolarContourFunctionWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    radiusFunction = new ExtWidget("Radius function",new ChoiceWidget(new Function1ArgWidgetFactory(element,"phi",1,scalar,1,scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"radiusFunction");
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
