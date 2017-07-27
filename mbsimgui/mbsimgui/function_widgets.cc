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
#include <QtGui>
#include "mainwindow.h"
#include <mbxmlutils/eval.h>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  class LimitedFunctionWidgetFactory : public WidgetFactory {
    public:
      LimitedFunctionWidgetFactory(Element *parent_) : parent(parent_) { }
      QString getName(int i=0) const { return "Limited function"; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return MBSIM%"LimitedFunction"; }
      QWidget* createWidget(int i=0);
      int getSize() const { return 1; }
    protected:
      Element *parent;
  };

  QWidget* LimitedFunctionWidgetFactory::createWidget(int i) {
    if(i==0)
      return new LimitedFunctionWidget(parent);
    return NULL;
  }

  class CoefficientWidgetFactory : public WidgetFactory {
    public:
      CoefficientWidgetFactory() { }
      Widget* createWidget(int i=0);
  };

  Widget* CoefficientWidgetFactory::createWidget(int i) {
    return new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5);
  }

  ConstantFunctionWidget::ConstantFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    a0 = new ExtWidget("a0",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"a0");
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

  LinearFunctionWidget::LinearFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a0 = new ExtWidget("a0",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a0");
    layout->addWidget(a0);

    a1 = new ExtWidget("a1",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"a1");
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

  QuadraticFunctionWidget::QuadraticFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a0 = new ExtWidget("a0",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a0");
    layout->addWidget(a0);

    a1 = new ExtWidget("a1",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a1");
    layout->addWidget(a1);

    a2 = new ExtWidget("a2",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"a2");
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

  PolynomFunctionWidget::PolynomFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a = new ExtWidget("Coefficients",new ChoiceWidget2(new VecSizeVarWidgetFactory(3,1,vector<QStringList>(3,QStringList()),vector<int>(3,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"coefficients");
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

  SinusoidalFunctionWidget::SinusoidalFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a = new ExtWidget("Amplitude",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"amplitude");
    layout->addWidget(a);

    f = new ExtWidget("Frequency",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frequency");
    layout->addWidget(f);

    p = new ExtWidget("Phase",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"phase");
    layout->addWidget(p);

    o = new ExtWidget("Offset",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"offset");
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

  ModuloFunctionWidget::ModuloFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    denom = new ExtWidget("Denominator",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"denominator");
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

  BoundedFunctionWidget::BoundedFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    lowerBound = new ExtWidget("Lower bound",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"lowerBound");
    layout->addWidget(lowerBound);

    upperBound = new ExtWidget("Upper bound",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"upperBound");
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

  VectorValuedFunctionWidget::VectorValuedFunctionWidget(Element *parent, int m, bool fixedSize) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    Function *dummy = new Function("NoName"); // Workaround for correct XML path. TODO: provide a consistent concept
    dummy->setParent(parent);
    functions = new ExtWidget("Components",new ListWidget(new ChoiceWidgetFactory(new FunctionWidgetFactory2(dummy),1),"Function",0,0),false,false,MBSIM%"components");
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

  CompositeFunctionWidget::CompositeFunctionWidget(WidgetFactory *factoryo, WidgetFactory *factoryi) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    fo = new ExtWidget("Outer function",new ChoiceWidget2(factoryo,QBoxLayout::TopToBottom,0),false,false,MBSIM%"outerFunction");
    layout->addWidget(fo);
    fi = new ExtWidget("Inner function",new ChoiceWidget2(factoryi,QBoxLayout::TopToBottom,0),false,false,MBSIM%"innerFunction");
    layout->addWidget(fi);
  }

  int CompositeFunctionWidget::getArg1Size() const {
    return static_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(fi->getWidget())->getWidget())->getArg1Size();
  }

  void CompositeFunctionWidget::updateWidget() {
    cout << "CompositeFunctionWidget::updateWidget() not yet implemented" << endl;
    // int size = static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(fo->getWidget())->getWidget())->getArg1Size();
    // static_cast<ChoiceWidget*>(fi->getWidget())->resize_(size,1);
  }

  void CompositeFunctionWidget::resize_(int m, int n) {
    static_cast<ChoiceWidget2*>(fo->getWidget())->resize_(m,n);
    static_cast<ChoiceWidget2*>(fi->getWidget())->resize_(static_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(fo->getWidget())->getWidget())->getArg1Size(),n);
  }

  DOMElement* CompositeFunctionWidget::initializeUsingXML(DOMElement *element) {
    fo->initializeUsingXML(element);
    fi->initializeUsingXML(element);
    return element;
  }

  DOMElement* CompositeFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    fo->writeXMLFile(ele0);
    fi->writeXMLFile(ele0);
    return ele0;
  }

  PiecewiseDefinedFunctionWidget::PiecewiseDefinedFunctionWidget(Element *parent, int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    functions = new ExtWidget("Limited functions",new ListWidget(new ChoiceWidgetFactory(new LimitedFunctionWidgetFactory(parent)),"Function",0,0),false,false,MBSIM%"limitedFunctions");
    layout->addWidget(functions);

    shiftAbscissa = new ExtWidget("Shift abscissa",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"shiftAbscissa");
    layout->addWidget(shiftAbscissa);

    shiftOrdinate = new ExtWidget("Shift ordinate",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"shiftOrdinate");
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

  LimitedFunctionWidget::LimitedFunctionWidget(Element *parent) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    function = new ExtWidget("Function",new ChoiceWidget2(new FunctionWidgetFactory2(parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"function");
    layout->addWidget(function);

    limit = new ExtWidget("Limit",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"limit");
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

  SymbolicFunctionWidget::SymbolicFunctionWidget(const QStringList &var, int m, int max, bool fixedSize) {
    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    for(int i=0; i<var.size(); i++) {
      argname.push_back(new ExtWidget("Name of argument "+QString::number(i+1),new TextWidget(var[i])));
      layout->addWidget(argname[i],i,0);

      argdim.push_back(new ExtWidget("Dimension of argument "+QString::number(i+1),new SpinBoxWidget(1,1,max)));
      if(var[i]!="t")
        layout->addWidget(argdim[i],i,1);
    }
    if(fixedSize)
      f = new ExtWidget("Function",new ChoiceWidget2(new VecWidgetFactory(m,vector<QStringList>(3,noUnitUnits()),vector<int>(3,0),false,false,false),QBoxLayout::RightToLeft,5),false,false,"");
    else
      f = new ExtWidget("Function",new ChoiceWidget2(new VecSizeVarWidgetFactory(m,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,0),false,false,false),QBoxLayout::RightToLeft,5),false,false,"");
    layout->addWidget(f,var.size(),0,1,2);
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

  void SymbolicFunctionWidget::resize_(int m, int n) {
    f->resize_(m,n);
  }

  DOMElement* SymbolicFunctionWidget::initializeUsingXML(DOMElement *element) {
    f->initializeUsingXML(element);
    for(size_t i=0; i<argname.size(); i++) {
      string str = "arg"+toStr(int(i+1));
      if(E(element)->hasAttribute(str))
        static_cast<TextWidget*>(argname[i]->getWidget())->setText(QString::fromStdString(E(element)->getAttribute(str.c_str())));
      str = "arg"+toStr(int(i+1))+"Dim";
      if(E(element)->hasAttribute(str))
        static_cast<SpinBoxWidget*>(argdim[i]->getWidget())->setValue(boost::lexical_cast<int>(E(element)->getAttribute(str.c_str())));
    }
    return element;
  }

  DOMElement* SymbolicFunctionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FunctionWidget::writeXMLFile(parent);
    for(size_t i=0; i<argname.size(); i++) {
      string istr = toStr(int(i+1));
      E(ele0)->setAttribute("arg"+istr, static_cast<TextWidget*>(argname[i]->getWidget())->getText().toStdString());
//      if(ext[i]=='V')
        E(ele0)->setAttribute("arg"+istr+"Dim",to_string(static_cast<SpinBoxWidget*>(argdim[i]->getWidget())->getValue()));
    }
    f->writeXMLFile(ele0);
    return ele0;
  }

  TabularFunctionWidget::TabularFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget2(new TabularFunctionWidgetFactory,QBoxLayout::TopToBottom,5);
    layout->addWidget(choice);

    choiceChanged();
    connect(choice,SIGNAL(widgetChanged()),this,SLOT(choiceChanged()));
    connect(choice,SIGNAL(widgetChanged()),this,SIGNAL(widgetChanged()));
  }

  void TabularFunctionWidget::choiceChanged() {
    if(choice->getIndex()==0) {
      updateWidget();
      connect(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    }
  }

  void TabularFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      ChoiceWidget2 *choice1_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      ChoiceWidget2 *choice2_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->resize_(static_cast<PhysicalVariableWidget*>(choice1_->getWidget())->rows(),static_cast<PhysicalVariableWidget*>(choice2_->getWidget())->getWidget()->cols());
    }
  }

  void TabularFunctionWidget::resize_(int m, int n) {
    if(choice->getIndex()==0) {
      ChoiceWidget2 *choice_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->resize_(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->rows(),m);
    }
    else {
      ChoiceWidget2 *choice_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(choice->getWidget())->getWidget());
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

  TwoDimensionalTabularFunctionWidget::TwoDimensionalTabularFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget2(new TwoDimensionalTabularFunctionWidgetFactory,QBoxLayout::TopToBottom,5);
    layout->addWidget(choice);

    choiceChanged();
    connect(choice,SIGNAL(widgetChanged()),this,SLOT(choiceChanged()));
  }

  void TwoDimensionalTabularFunctionWidget::choiceChanged() {
    if(choice->getIndex()==0) {
      connect(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
      connect(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    }
  }

  void TwoDimensionalTabularFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      ChoiceWidget2 *choice1_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      ChoiceWidget2 *choice2_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->getWidget());
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

  PiecewisePolynomFunctionWidget::PiecewisePolynomFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget2(new TabularFunctionWidgetFactory,QBoxLayout::TopToBottom,5);
    layout->addWidget(choice);

    vector<QString> list;
    list.push_back("\"cSplinePeriodic\"");
    list.push_back("\"cSplineNatural\"");
    list.push_back("\"piecewiseLinear\"");
    method = new ExtWidget("Interpolation method",new TextChoiceWidget(list,1,true),true,false,MBSIM%"interpolationMethod");
    layout->addWidget(method);

    choiceChanged();
    connect(choice,SIGNAL(widgetChanged()),this,SLOT(choiceChanged()));
    connect(choice,SIGNAL(widgetChanged()),this,SIGNAL(widgetChanged()));
  }

  void PiecewisePolynomFunctionWidget::choiceChanged() {
    if(choice->getIndex()==0) {
      updateWidget();
      connect(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    }
  }

  void PiecewisePolynomFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      ChoiceWidget2 *choice1_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      ChoiceWidget2 *choice2_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->resize_(static_cast<PhysicalVariableWidget*>(choice1_->getWidget())->rows(),static_cast<PhysicalVariableWidget*>(choice2_->getWidget())->getWidget()->cols());
    }
  }

  void PiecewisePolynomFunctionWidget::resize_(int m, int n) {
    if(choice->getIndex()==0) {
      ChoiceWidget2 *choice_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->resize_(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->rows(),m);
    }
    else {
      ChoiceWidget2 *choice_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(choice->getWidget())->getWidget());
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

  TwoDimensionalPiecewisePolynomFunctionWidget::TwoDimensionalPiecewisePolynomFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget2(new TwoDimensionalTabularFunctionWidgetFactory,QBoxLayout::TopToBottom,3);
    layout->addWidget(choice);

    vector<QString> list;
    list.push_back("\"cSplinePeriodic\"");
    list.push_back("\"cSplineNatural\"");
    list.push_back("\"piecewiseLinear\"");
    method = new ExtWidget("Interpolation method",new TextChoiceWidget(list,1,true),true);
    layout->addWidget(method);

    choiceChanged();
    connect(choice,SIGNAL(widgetChanged()),this,SLOT(choiceChanged()));
  }

  void TwoDimensionalPiecewisePolynomFunctionWidget::choiceChanged() {
    if(choice->getIndex()==0) {
      connect(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
      connect(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    }
  }

  void TwoDimensionalPiecewisePolynomFunctionWidget::updateWidget() {
    if(choice->getIndex()==0) {
      ChoiceWidget2 *choice1_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      ChoiceWidget2 *choice2_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(1))->getWidget());
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

  FourierFunctionWidget::FourierFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    f = new ExtWidget("Frequency",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"frequency");
    layout->addWidget(f);

    a0 = new ExtWidget("a0",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"a0");
    layout->addWidget(a0);

    choice = new ChoiceWidget2(new FourierFunctionWidgetFactory,QBoxLayout::TopToBottom,3);
    layout->addWidget(choice);

    amplitudePhaseAngleForm = new ExtWidget("Amplitude phase angle form",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"amplitudePhaseAngleForm");
    layout->addWidget(amplitudePhaseAngleForm);
  }

  void FourierFunctionWidget::resize_(int m, int n) {
    if(choice->getIndex()==0) {
      ChoiceWidget2 *choice_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(static_cast<ContainerWidget*>(choice->getWidget())->getWidget(0))->getWidget());
      if(choice_->getIndex()==0)
        choice->resize_(static_cast<VecSizeVarWidget*>(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->getWidget())->size(),m);
    }
    else {
      ChoiceWidget2 *choice_ = static_cast<ChoiceWidget2*>(static_cast<ExtWidget*>(choice->getWidget())->getWidget());
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

  BidirectionalFunctionWidget::BidirectionalFunctionWidget() {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    fn = new ExtWidget("Negative directional function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL),QBoxLayout::TopToBottom,0),false,false,MBSIM%"negativeDirectionalFunction");
    layout->addWidget(fn);
    fp = new ExtWidget("Positive directional function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL),QBoxLayout::TopToBottom,0),false,false,MBSIM%"positiveDirectionalFunction");
    layout->addWidget(fp);
  }

  void BidirectionalFunctionWidget::resize_(int m, int n) {
    static_cast<ChoiceWidget2*>(fn->getWidget())->resize_(m,n);
    static_cast<ChoiceWidget2*>(fp->getWidget())->resize_(m,n);
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

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    f = new ExtWidget("Function",new ChoiceWidget2(factoryf,QBoxLayout::TopToBottom,0),false,false,MBSIM%"function");
    layout->addWidget(f);

    r = new ExtWidget("Continuation rule",new ChoiceWidget2(factoryr,QBoxLayout::TopToBottom,0),false,false,MBSIM%"continuationRule");
    layout->addWidget(r);
  }

  void ContinuedFunctionWidget::resize_(int m, int n) {
    static_cast<ChoiceWidget2*>(f->getWidget())->resize_(m,n);
    static_cast<ChoiceWidget2*>(r->getWidget())->resize_(n,n);
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
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    c = new ExtWidget("Stiffness coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessCoefficient");
    layout->addWidget(c);

    d = new ExtWidget("Damping coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dampingCoefficient");
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

  NonlinearSpringDamperForceWidget::NonlinearSpringDamperForceWidget(Element *parent) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    s = new ExtWidget("Force deflection function",new ChoiceWidget2(new FunctionWidgetFactory2(parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceDeflectionFunction");
    layout->addWidget(s);

    sd = new ExtWidget("Force velocity function",new ChoiceWidget2(new FunctionWidgetFactory2(parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceVelocityFunction");
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

  LinearElasticFunctionWidget::LinearElasticFunctionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    K = new ExtWidget("Generalized stiffness matrix",new ChoiceWidget2(new SymMatWidgetFactory(getEye<QString>(3,3,"0","0"),vector<QStringList>(3),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessMatrix");
    layout->addWidget(K);

    D = new ExtWidget("Generalized damping matrix",new ChoiceWidget2(new SymMatWidgetFactory(getEye<QString>(3,3,"0","0"),vector<QStringList>(3),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dampingMatrix");
    layout->addWidget(D);
  }

  void LinearElasticFunctionWidget::resize_(int m, int n) {
    K->resize_(m,n);
    D->resize_(m,n);
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
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    c = new ExtWidget("Stiffness coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessCoefficient");
    layout->addWidget(c);

    d = new ExtWidget("Damping coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dampingCoefficient");
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
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    c = new ExtWidget("Stiffness coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"stiffnessCoefficient");
    layout->addWidget(c);

    d = new ExtWidget("Damping coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dampingCoefficient");
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
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    gd = new ExtWidget("Marginal velocity",new ChoiceWidget2(new ScalarWidgetFactory("0.01",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"marginalVelocity");
    layout->addWidget(gd);

    mu = new ExtWidget("Friction coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"frictionCoefficient");
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
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    gd = new ExtWidget("Marginal velocity",new ChoiceWidget2(new ScalarWidgetFactory("0.01",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"marginalVelocity");
    layout->addWidget(gd);

    mu = new ExtWidget("Friction coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"frictionCoefficient");
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

  SignalFunctionWidget::SignalFunctionWidget(Element *element) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    dummy = new Function("NoName"); // Workaround for correct XML path. TODO: provide a consistent concept
    dummy->setParent(element);
    sRef = new ExtWidget("Return signal",new SignalOfReferenceWidget(dummy,0),false,false,MBSIMCONTROL%"returnSignal");
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

  PolarContourFunctionWidget::PolarContourFunctionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    radiusFunction = new ExtWidget("Radius function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL),QBoxLayout::TopToBottom,0),false,false,MBSIM%"radiusFunction");
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

}
