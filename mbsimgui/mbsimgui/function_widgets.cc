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

SymbolicFunction1Widget::SymbolicFunction1Widget(const QString &ext) : Function1Widget(ext) {
  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  for(int i=1; i<ext.size(); i++) {
    argname.push_back(new ExtWidget("Name of argument "+QString::number(i),new TextWidget(QString("x")+QString::number(i))));
    layout->addWidget(argname[i-1],i-1,0);

    argdim.push_back(new ExtWidget("Dimension of argument "+QString::number(i),new TextWidget("1")));
    if(ext[i]=='V')
      layout->addWidget(argdim[i-1],i-1,1);
  }
  f = new ExtWidget("Function",new OctaveExpressionWidget);
  layout->addWidget(f,ext.size()-1,0,1,2);
}

int SymbolicFunction1Widget::getArgDim() const {
  return static_cast<TextWidget*>(argdim[0]->getWidget())->getText().toInt();
}

ConstantFunction1Widget::ConstantFunction1Widget(const QString &ext, int n) : Function1Widget(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(n,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  c = new ExtWidget("Value",new ExtPhysicalVarWidget(input));
  layout->addWidget(c);
}

void ConstantFunction1Widget::resize(int m, int n) {
  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m)
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize(m);
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

void QuadraticFunction1Widget::resize(int m, int n) {
  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a1->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a2->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize(m);
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

void SinusFunction1Widget::resize(int m, int n) {
  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(f->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(p->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize(m);
    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(o->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize(m);
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

void SummationFunction1Widget::resize(int m, int n) {
  for(int i=0; i<functionChoice.size(); i++)
   functionChoice[i]->resize(m,n);
}

void SummationFunction1Widget::updateList() {
  for(int i=0; i<functionList->count(); i++)
    functionList->item(i)->setText(functionChoice[i]->getFunction()->getType());
}

void SummationFunction1Widget::addFunction() {
  int i = functionChoice.size();
  functionChoice.push_back(new Function1ChoiceWidget(true,n));
  if(i>0)
    functionChoice[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  functionList->addItem("Undefined");
  connect(functionChoice[i],SIGNAL(functionChanged()),this,SLOT(updateList()));
  connect(functionChoice[i],SIGNAL(resize()),this,SIGNAL(resize()));
  stackedWidget->addWidget(functionChoice[i]);
  emit resize();
  emit updateList();
}

void SummationFunction1Widget::removeFunction() {
  int i = functionList->currentRow();
  stackedWidget->removeWidget(functionChoice[i]);
  delete functionChoice[i];
  functionChoice.erase(functionChoice.begin()+i);
  delete functionList->takeItem(i);
}

SymbolicFunction2Widget::SymbolicFunction2Widget(const QString &ext) : Function2Widget(ext) {
  QGridLayout *layout = new QGridLayout;
  layout->setMargin(0);
  setLayout(layout);
  for(int i=1; i<ext.size(); i++) {
    argname.push_back(new ExtWidget("Name of argument "+QString::number(i),new TextWidget(QString("x")+QString::number(i))));
    layout->addWidget(argname[i-1],i-1,0);

    argdim.push_back(new ExtWidget("Dimension of argument "+QString::number(i),new TextWidget("1")));
    if(ext[i]=='V')
      layout->addWidget(argdim[i-1],i-1,1);
  }
  f = new ExtWidget("Function",new OctaveExpressionWidget);
  layout->addWidget(f,ext.size()-1,0,1,2);
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

Function1ChoiceWidget::Function1ChoiceWidget(bool withFactor, int n_, const QString& ext_) : factor(0), n(n_), ext(ext_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  if(withFactor) {
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),noUnitUnits(),1));
    factor = new ExtWidget("Factor",new ExtPhysicalVarWidget(input));
    layout->addWidget(factor);
  }

  comboBox = new QComboBox;
  comboBox->addItem(tr("Constant function"));
  comboBox->addItem(tr("Quadratic function"));
  comboBox->addItem(tr("Sinus function"));
  comboBox->addItem(tr("Tabular function"));
  comboBox->addItem(tr("Summation function"));
  comboBox->addItem(tr("Symbolic function"));
  layout->addWidget(comboBox);
  stackedWidget = new QStackedWidget;
  Function1Widget *function;
  function = new ConstantFunction1Widget(ext,n);
  stackedWidget->addWidget(function);
  function = new QuadraticFunction1Widget(n);
  function->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(function);
  function = new SinusFunction1Widget(n);     
  function->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(function);
  function = new TabularFunction1Widget(n);   
  function->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(function);
  function = new SummationFunction1Widget(n);
  connect(function,SIGNAL(resize()),this,SIGNAL(resize())); 
  function->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(function);
  function = new SymbolicFunction1Widget(ext);
  function->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(function);
  layout->addWidget(stackedWidget);
  connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(defineFunction(int)));
}

void Function1ChoiceWidget::resize(int m, int n) {
  getFunction()->resize(m,n);
}

Function1Widget* Function1ChoiceWidget::getFunction() {
  return static_cast<Function1Widget*>(stackedWidget->currentWidget());
}

Function1Widget* Function1ChoiceWidget::getFunction(int i) {
  return static_cast<Function1Widget*>(stackedWidget->widget(i));
}

void Function1ChoiceWidget::defineFunction(int index) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(index);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
  emit functionChanged();
  emit resize();
}

Function2ChoiceWidget::Function2ChoiceWidget(const QString& ext_) : ext(ext_) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Linear spring damper force"));
  comboBox->addItem(tr("Symbolic function"));
  layout->addWidget(comboBox);
  stackedWidget = new QStackedWidget;
  Function2Widget *function;
  function = new LinearSpringDamperForceWidget;
  stackedWidget->addWidget(function);
  function = new SymbolicFunction2Widget(ext);
  function->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->addWidget(function);
  layout->addWidget(stackedWidget);
  connect(comboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(defineFunction(int)));
}

void Function2ChoiceWidget::resize(int m, int n) {
  getFunction()->resize(m,n);
}

Function2Widget* Function2ChoiceWidget::getFunction() {
  return static_cast<Function2Widget*>(stackedWidget->currentWidget());
}

Function2Widget* Function2ChoiceWidget::getFunction(int i) {
  return static_cast<Function2Widget*>(stackedWidget->widget(i));
}

void Function2ChoiceWidget::defineFunction(int index) {
  if (stackedWidget->currentWidget() !=0)
    stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  stackedWidget->setCurrentIndex(index);
  stackedWidget->currentWidget()->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  adjustSize();
  emit functionChanged();
  emit resize();
}
