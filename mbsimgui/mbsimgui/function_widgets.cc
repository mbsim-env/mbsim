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
#include "octaveutils.h"
#include <QtGui>

using namespace std;

void DifferentiableFunction1Widget::setDerivative(Function1Widget *diff,size_t degree) { 
  derivatives.resize(max(derivatives.size(),degree+1)); 
  derivatives[degree]=diff; 
}

SymbolicFunction1Widget::SymbolicFunction1Widget(const QStringList &var, int max) : Function1Widget() {
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
  f = new ExtWidget("Function",new OctaveExpressionWidget);
  layout->addWidget(f,var.size(),0,1,2);
}

int SymbolicFunction1Widget::getArgDim() const {
  return static_cast<SpinBoxWidget*>(argdim[0]->getWidget())->getValue();
}

ConstantFunction1Widget::ConstantFunction1Widget(bool vec, int n) : Function1Widget() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  if(vec)
    input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  c = new ExtWidget("Value",new ExtPhysicalVarWidget(input));
  layout->addWidget(c);
}

void ConstantFunction1Widget::resize_(int m, int n) {
  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m)
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
}

QuadraticFunction1Widget::QuadraticFunction1Widget(int n) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  a0 = new ExtWidget("a0",new ExtPhysicalVarWidget(input));
  layout->addWidget(a0);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  a1 = new ExtWidget("a1",new ExtPhysicalVarWidget(input));
  layout->addWidget(a1);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  a2 = new ExtWidget("a2",new ExtPhysicalVarWidget(input));
  layout->addWidget(a2);
}

void QuadraticFunction1Widget::resize_(int m, int n) {
  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a1->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a2->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
  }
}

SinusFunction1Widget::SinusFunction1Widget(int n) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  a = new ExtWidget("Amplitude",new ExtPhysicalVarWidget(input));
  layout->addWidget(a);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  f = new ExtWidget("Frequency",new ExtPhysicalVarWidget(input));
  layout->addWidget(f);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  p = new ExtWidget("Phase",new ExtPhysicalVarWidget(input));
  layout->addWidget(p);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  o = new ExtWidget("Offset",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(o);
}

void SinusFunction1Widget::resize_(int m, int n) {
  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(f->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(p->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(o->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
  }
}

TabularFunction1Widget::TabularFunction1Widget(int n) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<QWidget*> choiceWidget;
  vector<QString> name;
  name.push_back("x and y");
  name.push_back("xy");
  WidgetContainer *widgetContainer = new WidgetContainer;
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecFromFileWidget,QStringList(),0));
  widgetContainer->addWidget(new ExtWidget("x",new ExtPhysicalVarWidget(input)));

  input.clear();
  input.push_back(new PhysicalVariableWidget(new MatFromFileWidget,QStringList(),0));
  widgetContainer->addWidget(new ExtWidget("y",new ExtPhysicalVarWidget(input)));

  choiceWidget.push_back(widgetContainer);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new MatFromFileWidget,QStringList(),0));
  choiceWidget.push_back(new ExtWidget("xy",new ExtPhysicalVarWidget(input)));

  choice = new WidgetChoiceWidget(name,choiceWidget);
  layout->addWidget(choice);
}

SummationFunction1Widget::SummationFunction1Widget(int n_) : n(n_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  functionList = new QListWidget;
  functionList->setContextMenuPolicy (Qt::CustomContextMenu);
  functionList->setMinimumWidth(functionList->sizeHint().width()/3);
  functionList->setMaximumWidth(functionList->sizeHint().width()/3);
  layout->addWidget(functionList);
  stackedWidget = new QStackedWidget;
  connect(functionList,SIGNAL(currentRowChanged(int)),this,SLOT(changeCurrent(int)));
  connect(functionList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  layout->addWidget(stackedWidget,0,Qt::AlignTop);
}

void SummationFunction1Widget::changeCurrent(int idx) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(idx);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
}

void SummationFunction1Widget::openContextMenu(const QPoint &pos) {
  if(functionList->itemAt(pos)) {
    QMenu menu(this);
    QAction *add = new QAction(tr("Remove"), this);
    connect(add, SIGNAL(triggered()), this, SLOT(removeFunction()));
    menu.addAction(add);
    menu.exec(QCursor::pos());
  }
  else {
    QMenu menu(this);
    QAction *add = new QAction(tr("Add"), this);
    connect(add, SIGNAL(triggered()), this, SLOT(addFunction()));
    menu.addAction(add);
    menu.exec(QCursor::pos());
  }
}

void SummationFunction1Widget::resize_(int m, int n) {
  for(int i=0; i<stackedWidget->count(); i++)
   static_cast<Widget*>(static_cast<WidgetContainer*>(stackedWidget->widget(i))->getWidget(0))->resize_(m,n);
}

void SummationFunction1Widget::updateList() {
  for(int i=0; i<functionList->count(); i++)
    functionList->item(i)->setText(static_cast<GeneralChoiceWidget*>(static_cast<WidgetContainer*>(stackedWidget->widget(i))->getWidget(0))->getWidget()->getType());
}

void SummationFunction1Widget::addFunction() {
  int i = stackedWidget->count();

  WidgetContainer *widgetContainer = new WidgetContainer;
  vector<Widget*> widget;
  vector<QString> name;
  widget.push_back(new ConstantFunction1Widget(true,n));
  name.push_back("Constant function");
  widget.push_back(new QuadraticFunction1Widget(n));
  name.push_back("Quadratic function");
  widget.push_back(new SinusFunction1Widget(n));
  name.push_back("Sinus function");
  widget.push_back(new TabularFunction1Widget(n));
  name.push_back("Tabular function");
  widget.push_back(new SummationFunction1Widget(n));
  name.push_back("Summation function");
  QStringList var;
  var << "t";
  widget.push_back(new SymbolicFunction1Widget(var));
  name.push_back("Symbolic function");
  widgetContainer->addWidget(new GeneralChoiceWidget(widget,name));

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),noUnitUnits(),1));
  widgetContainer->addWidget(new ExtWidget("Factor",new ExtPhysicalVarWidget(input)));

  functionList->addItem("Undefined");

  stackedWidget->addWidget(widgetContainer);
  if(i>0)
    widgetContainer->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

  connect(static_cast<GeneralChoiceWidget*>(static_cast<WidgetContainer*>(stackedWidget->widget(i))->getWidget(0)),SIGNAL(widgetChanged()),this,SLOT(updateList()));
  connect(static_cast<GeneralChoiceWidget*>(static_cast<WidgetContainer*>(stackedWidget->widget(i))->getWidget(0)),SIGNAL(resize_()),this,SIGNAL(resize_()));

  emit resize_();
  emit updateList();
}

void SummationFunction1Widget::removeFunction() {
  int i = functionList->currentRow();
  delete stackedWidget->widget(i);
  stackedWidget->removeWidget(stackedWidget->widget(i));
  //functionChoice.erase(functionChoice.begin()+i);
  //delete factor[i];
  //factor.erase(factor.begin()+i);
  delete functionList->takeItem(i);
}

SymbolicFunction2Widget::SymbolicFunction2Widget(const QStringList &var, int max) : Function2Widget() {
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
  f = new ExtWidget("Function",new OctaveExpressionWidget);
  layout->addWidget(f,var.size(),0,1,2);
}

int SymbolicFunction2Widget::getArgDim(int i) const {
  return static_cast<SpinBoxWidget*>(argdim[i]->getWidget())->getValue();
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
