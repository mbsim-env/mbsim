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
#include "function_widget_factory.h"
#include <QtGui>
#include "mainwindow.h"
#include <mbxmlutils/octeval.h>

using namespace std;

namespace MBSimGUI {

  class LimitedFunctionWidgetFactory : public WidgetFactory {
    public:
      LimitedFunctionWidgetFactory(WidgetFactory *factory_) : factory(factory_) { }
      Widget* createWidget(int i=0);
    protected:
      WidgetFactory *factory;
  };

  Widget *LimitedFunctionWidgetFactory::createWidget(int i) {
    ContainerWidget *widget = new ContainerWidget;
    widget->addWidget(new ExtWidget("Function",new ChoiceWidget2(factory)));

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),QStringList(),1));
    widget->addWidget(new ExtWidget("Limit",new ExtPhysicalVarWidget(input)));
    return widget;
  }

  //class LimitedFunctionWidgetFactory : public WidgetFactory {
  //  public:
  //    LimitedFunctionWidgetFactory() { }
  //    Widget* createWidget(int i=0);
  //};
  //
  //Widget* LimitedFunctionWidgetFactory::createWidget(int i) {
  //  FunctionWidgetFactory factory("PiecewiseDefinedFunction","VS",1);
  //
  //  ContainerWidget *widget = new ContainerWidget;
  //  widget->addWidget(new ExtWidget("Function",factory.createWidget()));
  //
  //  vector<PhysicalVariableWidget*> input;
  //  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),noUnitUnits(),1));
  //  widget->addWidget(new ExtWidget("Limit",new ExtPhysicalVarWidget(input)));
  //  return widget;
  //}

  class CoefficientWidgetFactory : public WidgetFactory {
    public:
      CoefficientWidgetFactory() { }
      Widget* createWidget(int i=0);
  };

  Widget* CoefficientWidgetFactory::createWidget(int i) {
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),noUnitUnits(),1));
    return new ExtPhysicalVarWidget(input);
  }

  ConstantFunctionWidget::ConstantFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    a0 = new ExtWidget("Value",new ExtPhysicalVarWidget(input));
    layout->addWidget(a0);
  }

  void ConstantFunctionWidget::resize_(int m, int n) {
    //  if(ext[0]=='V') {
    //    if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
    //      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    //    }
    //  }
  }

  LinearFunctionWidget::LinearFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    a0 = new ExtWidget("a0",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(a0);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),QStringList(),0));
    a1 = new ExtWidget("a1",new ExtPhysicalVarWidget(input));
    layout->addWidget(a1);
  }

  void LinearFunctionWidget::resize_(int m, int n) {
    //  if(ext[0]=='V') {
    //    if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a1->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
    //      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a1->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    //      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    //    }
    //  }
  }

  QuadraticFunctionWidget::QuadraticFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    a0 = new ExtWidget("a0",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(a0);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    a1 = new ExtWidget("a1",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(a1);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    a2 = new ExtWidget("a2",new ExtPhysicalVarWidget(input));
    layout->addWidget(a2);
  }

  void QuadraticFunctionWidget::resize_(int m, int n) {
    //  if(ext[0]=='V') {
    //    if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
    //      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    //      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a1->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    //      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a2->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    //    }
    //  }
  }

  PolynomFunctionWidget::PolynomFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    a = new ExtWidget("Coefficients",new ChoiceWidget2(new VecSizeVarWidgetFactory(3,vector<QStringList>(3,QStringList()))));
    layout->addWidget(a);
  }

  void PolynomFunctionWidget::resize_(int m, int n) {
    //  if(ext[0]=='V') {
    //    if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
    //      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    //    }
    //  }
  }

  SinusoidalFunctionWidget::SinusoidalFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    a = new ExtWidget("Amplitude",new ExtPhysicalVarWidget(input));
    layout->addWidget(a);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    f = new ExtWidget("Frequency",new ExtPhysicalVarWidget(input));
    layout->addWidget(f);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    p = new ExtWidget("Phase",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(p);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    o = new ExtWidget("Offset",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(o);
  }

  ModuloFunctionWidget::ModuloFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    denom = new ExtWidget("Denominator",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(denom);
  }

  AdditionFunctionWidget::AdditionFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    f1 = new ExtWidget("First summand",new ChoiceWidget2(new FunctionWidgetFactory2));
    layout->addWidget(f1);
    f2 = new ExtWidget("Second summand",new ChoiceWidget2(new FunctionWidgetFactory2));
    layout->addWidget(f2);
  }

  void AdditionFunctionWidget::resize_(int m, int n) {
    static_cast<ChoiceWidget2*>(f1->getWidget())->resize_(m,n);
    static_cast<ChoiceWidget2*>(f2->getWidget())->resize_(m,n);
  }

  MultiplicationFunctionWidget::MultiplicationFunctionWidget(int m) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    f1 = new ExtWidget("First factor",new ChoiceWidget2(new FunctionWidgetFactory2));
    layout->addWidget(f1);
    f2 = new ExtWidget("Second factor",new ChoiceWidget2(new FunctionWidgetFactory2));
    layout->addWidget(f2);
  }

  void MultiplicationFunctionWidget::resize_(int m, int n) {
//    functions->resize_(m,n);
  }

  VectorValuedFunctionWidget::VectorValuedFunctionWidget(int m, bool fixedSize) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    functions = new ExtWidget("Components",new ListWidget(new ChoiceWidgetFactory(new FunctionWidgetFactory2),"Function",m,1));
    layout->addWidget(functions);
  }

  void VectorValuedFunctionWidget::resize_(int m, int n) {
    static_cast<ListWidget*>(functions->getWidget())->setSize(m);
  }

  NestedFunctionWidget::NestedFunctionWidget(WidgetFactory *factoryo, WidgetFactory *factoryi) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    fo = new ExtWidget("Outer function",new ChoiceWidget2(factoryo));
    layout->addWidget(fo);
    fi = new ExtWidget("Inner function",new ChoiceWidget2(factoryi));
    layout->addWidget(fi);
  }

  int NestedFunctionWidget::getArg1Size() const {
    return static_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(fi->getWidget())->getWidget())->getArg1Size();
  }

  void NestedFunctionWidget::resizeVariables() {
    cout << "NestedFunctionWidget::resizeVariables() not yet implemented" << endl;
    // int size = static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(fo->getWidget())->getWidget())->getArg1Size();
    // static_cast<ChoiceWidget*>(fi->getWidget())->resize_(size,1);
  }

  void NestedFunctionWidget::resize_(int m, int n) {
    static_cast<ChoiceWidget2*>(fo->getWidget())->resize_(m,n);
    static_cast<ChoiceWidget2*>(fi->getWidget())->resize_(static_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(fo->getWidget())->getWidget())->getArg1Size(),n);
  }

  PiecewiseDefinedFunctionWidget::PiecewiseDefinedFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    functions = new ExtWidget("Limited functions",new ListWidget(new LimitedFunctionWidgetFactory(new FunctionWidgetFactory2),"Function",n,1));
    layout->addWidget(functions);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
    contDiff = new ExtWidget("Continously differentiable",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(contDiff);
  }

  void PiecewiseDefinedFunctionWidget::resize_(int m, int n) {
    functions->resize_(m,n);
  }

  SymbolicFunctionWidget::SymbolicFunctionWidget(const QStringList &var, int m, int max) {
    QGridLayout *layout = new QGridLayout;
    layout->setMargin(0);
    setLayout(layout);
    for(int i=0; i<var.size(); i++) {
      argname.push_back(new ExtWidget("Name of argument "+QString::number(i+1),new TextWidget(var[i])));
      layout->addWidget(argname[i],i,0);

      argdim.push_back(new ExtWidget("Dimension of argument "+QString::number(i+1),new SpinBoxWidget(1,1,max)));
      if(var[i]!="t") {
        connect(argdim[i]->getWidget(),SIGNAL(valueChanged(int)),this,SIGNAL(arg1SizeChanged(int)));
        layout->addWidget(argdim[i],i,1);
      }
    }
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new VecWidget(m),QStringList(),1));
    f = new ExtWidget("Function",new ExtPhysicalVarWidget(input));
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
    static_cast<VecWidget*>(static_cast<PhysicalVariableWidget*>(static_cast<ExtPhysicalVarWidget*>(f->getWidget())->getCurrentPhysicalVariableWidget())->getWidget())->resize_(m);
  }

  TabularFunctionWidget::TabularFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget2(new TabularFunctionWidgetFactory);
    layout->addWidget(choice);
  }

  void TabularFunctionWidget::resize_(int m, int n) {
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

  TwoDimensionalTabularFunctionWidget::TwoDimensionalTabularFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    x = new ExtWidget("X",new ChoiceWidget2(new VecSizeVarWidgetFactory(3,vector<QStringList>(3,QStringList()))));
    layout->addWidget(x);
    y = new ExtWidget("Y",new ChoiceWidget2(new VecSizeVarWidgetFactory(3,vector<QStringList>(3,QStringList()))));
    layout->addWidget(y);
    xy = new ExtWidget("XY",new ChoiceWidget2(new MatWidgetFactory(getScalars<QString>(3,3,"0"),vector<QStringList>(3,QStringList()),vector<int>(3,0))));
    layout->addWidget(xy);
  }

  void TwoDimensionalTabularFunctionWidget::resize_(int m, int n) {
    //ChoiceWidget2 *choice_ = static_cast<ChoiceWidget2*>(x->getWidget());
    //if(choice_->getIndex()==0)
    //  x->resize_(static_cast<VecSizeVarWidget*>(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->getWidget())->size(),m);
    //choice_ = static_cast<ChoiceWidget2*>(y->getWidget());
    //if(choice_->getIndex()==0)
    //  y->resize_(static_cast<VecSizeVarWidget*>(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->getWidget())->size(),m);
    //choice_ = static_cast<ChoiceWidget2*>(xy->getWidget());
    //if(choice_->getIndex()==0)
    //  xy->resize_(static_cast<MatRowsVarWidget*>(static_cast<PhysicalVariableWidget*>(choice_->getWidget())->getWidget())->rows(),m+1);
  }

  PiecewisePolynomFunctionWidget::PiecewisePolynomFunctionWidget(int n) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    choice = new ChoiceWidget2(new TabularFunctionWidgetFactory);
    layout->addWidget(choice);

    vector<QString> list;
    list.push_back("\"cSplinePeriodic\"");
    list.push_back("\"cSplineNatural\"");
    list.push_back("\"piecewiseLinear\"");
    method = new ExtWidget("Interpolation method",new TextChoiceWidget(list,1,true),true);
    layout->addWidget(method);
  }

  void PiecewisePolynomFunctionWidget::resize_(int m, int n) {
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

  LinearSpringDamperForceWidget::LinearSpringDamperForceWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),stiffnessUnits(),1));
    c = new ExtWidget("Stiffness coefficient",new ExtPhysicalVarWidget(input));
    layout->addWidget(c);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),dampingUnits(),0));
    d = new ExtWidget("Damping coefficient",new ExtPhysicalVarWidget(input));
    layout->addWidget(d);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),lengthUnits(),4));
    l0 = new ExtWidget("Unloaded length",new ExtPhysicalVarWidget(input));
    layout->addWidget(l0);
  }

  NonlinearSpringDamperForceWidget::NonlinearSpringDamperForceWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    g = new ExtWidget("Distance function",new ChoiceWidget2(new FunctionWidgetFactory2));
    layout->addWidget(g);

    gd = new ExtWidget("Velocity function",new ChoiceWidget2(new FunctionWidgetFactory2));
    layout->addWidget(gd);
  }

  LinearRegularizedBilateralConstraintWidget::LinearRegularizedBilateralConstraintWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),stiffnessUnits(),1));
    c = new ExtWidget("Stiffness coefficient",new ExtPhysicalVarWidget(input));
    layout->addWidget(c);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),dampingUnits(),0));
    d = new ExtWidget("Damping coefficient",new ExtPhysicalVarWidget(input));
    layout->addWidget(d);
  }

  LinearRegularizedUnilateralConstraintWidget::LinearRegularizedUnilateralConstraintWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),stiffnessUnits(),1));
    c = new ExtWidget("Stiffness coefficient",new ExtPhysicalVarWidget(input));
    layout->addWidget(c);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),dampingUnits(),0));
    d = new ExtWidget("Damping coefficient",new ExtPhysicalVarWidget(input));
    layout->addWidget(d);
  }

  LinearRegularizedCoulombFrictionWidget::LinearRegularizedCoulombFrictionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.01"),velocityUnits(),0));
    gd = new ExtWidget("Marginal velocity",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(gd);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
    mu = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
    layout->addWidget(mu);
  }

}
