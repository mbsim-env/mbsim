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
#include "function_widgets.h"
#include "utils.h"
#include "octaveutils.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include <QtGui>

using namespace std;

void DifferentiableFunction1::setDerivative(Function1 *diff,size_t degree) { 
  derivatives.resize(max(derivatives.size(),degree+1)); 
  derivatives[degree]=diff; 
}

ConstantFunction1::ConstantFunction1(const QString &ext) : Function1(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),QStringList(),0));
  c = new ExtPhysicalVarWidget(input),"VS";  
  ExtWidget *extWidget = new ExtWidget("Value",c);
  layout->addWidget(extWidget);
}

void ConstantFunction1::resize(int m, int n) {
  if(((VecWidget*)c->getPhysicalStringWidget(0)->getWidget())->size() != m)
    ((VecWidget*)c->getPhysicalStringWidget(0)->getWidget())->resize(m);
}

QuadraticFunction1::QuadraticFunction1() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  widget.push_back(new ExtWidget("a0",var[var.size()-1]));
  layout->addWidget(widget[widget.size()-1]);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  widget.push_back(new ExtWidget("a1",var[var.size()-1]));
  layout->addWidget(widget[widget.size()-1]);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  widget.push_back(new ExtWidget("a2",var[var.size()-1]));
  layout->addWidget(widget[widget.size()-1]);
}

void QuadraticFunction1::resize(int m, int n) {
  for(unsigned int i=0; i<var.size(); i++)
    if(((VecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->size() != m)
      ((VecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->resize(m);
}

SinusFunction1::SinusFunction1() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  widget.push_back(new ExtWidget("Amplitude",var[var.size()-1]));
  layout->addWidget(widget[widget.size()-1]);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  widget.push_back(new ExtWidget("Frequency",var[var.size()-1]));
  layout->addWidget(widget[widget.size()-1]);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  widget.push_back(new ExtWidget("Phase",var[var.size()-1]));
  layout->addWidget(widget[widget.size()-1]);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));  
  widget.push_back(new ExtWidget("Offset",var[var.size()-1],true));
  layout->addWidget(widget[widget.size()-1]);
}

void SinusFunction1::resize(int m, int n) {
  for(unsigned int i=0; i<var.size(); i++)
    if(((VecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->size() != m)
      ((VecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->resize(m);
}

TabularFunction1::TabularFunction1() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<QWidget*> choiceWidget;
  vector<string> name;
  name.push_back("x and y");
  name.push_back("xy");
  WidgetContainer *widgetContainer = new WidgetContainer;
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecFromFileWidget,QStringList(),0));
  widgetContainer->addWidget(new ExtWidget("x",new ExtPhysicalVarWidget(input)));

  input.clear();
  input.push_back(new PhysicalStringWidget(new MatFromFileWidget,QStringList(),0));
  widgetContainer->addWidget(new ExtWidget("y",new ExtPhysicalVarWidget(input)));

  choiceWidget.push_back(widgetContainer);

  input.clear();
  input.push_back(new PhysicalStringWidget(new MatFromFileWidget,QStringList(),0));
  choiceWidget.push_back(new ExtWidget("xy",new ExtPhysicalVarWidget(input)));

  widget = new WidgetChoiceWidget(name,choiceWidget);
  layout->addWidget(widget);
}

SummationFunction1::SummationFunction1() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  functionList = new QListWidget;
  functionList->setContextMenuPolicy (Qt::CustomContextMenu);
  functionList->setMinimumWidth(functionList->sizeHint().width()/3);
  functionList->setMaximumWidth(functionList->sizeHint().width()/3);
  layout->addWidget(functionList);
  stackedWidget = new QStackedWidget;
  connect(functionList,SIGNAL(currentRowChanged(int)),stackedWidget,SLOT(setCurrentIndex(int)));
  connect(functionList,SIGNAL(customContextMenuRequested(const QPoint &)),this,SLOT(openContextMenu(const QPoint &)));
  layout->addWidget(stackedWidget,0,Qt::AlignTop);
}

void SummationFunction1::openContextMenu(const QPoint &pos) {
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

void SummationFunction1::resize(int m, int n) {
  for(int i=0; i<functionChoice.size(); i++)
   functionChoice[i]->resize(m,n);
}

void SummationFunction1::updateList() {
  for(int i=0; i<functionList->count(); i++)
    functionList->item(i)->setText(functionChoice[i]->getFunction()->getType());
}

void SummationFunction1::addFunction() {
  int i = functionChoice.size();
  functionChoice.push_back(new Function1ChoiceWidget(true));
  functionList->addItem("Undefined");
  connect(functionChoice[i],SIGNAL(functionChanged()),this,SLOT(updateList()));
  connect(functionChoice[i],SIGNAL(resize()),this,SIGNAL(resize()));
  stackedWidget->addWidget(functionChoice[i]);
  emit resize();
  emit updateList();
}

void SummationFunction1::removeFunction() {
  int i = functionList->currentRow();
  stackedWidget->removeWidget(functionChoice[i]);
  delete functionChoice[i];
  functionChoice.erase(functionChoice.begin()+i);
  delete functionList->takeItem(i);
}

LinearSpringDamperForce::LinearSpringDamperForce() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),stiffnessUnits(),1));
  var.push_back(new ExtPhysicalVarWidget(input));
  ExtWidget *extWidget = new ExtWidget("Stiffness coefficient",var[var.size()-1]);
  layout->addWidget(extWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),dampingUnits(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  extWidget = new ExtWidget("Damping coefficient",var[var.size()-1]);
  layout->addWidget(extWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),lengthUnits(),4));
  var.push_back(new ExtPhysicalVarWidget(input));
  extWidget = new ExtWidget("Unloaded length",var[var.size()-1]);
  layout->addWidget(extWidget);
}

LinearRegularizedBilateralConstraint::LinearRegularizedBilateralConstraint() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),stiffnessUnits(),1));
  var.push_back(new ExtWidget("Stiffness coefficient",new ExtPhysicalVarWidget(input)));

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),dampingUnits(),0));
  var.push_back(new ExtWidget("Damping coefficient",new ExtPhysicalVarWidget(input)));

  layout->addWidget(var[0]);
  layout->addWidget(var[1]);
}

LinearRegularizedUnilateralConstraint::LinearRegularizedUnilateralConstraint() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),stiffnessUnits(),1));
  var.push_back(new ExtWidget("Stiffness coefficient",new ExtPhysicalVarWidget(input)));

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),dampingUnits(),0));
  var.push_back(new ExtWidget("Damping coefficient",new ExtPhysicalVarWidget(input)));

  layout->addWidget(var[0]);
  layout->addWidget(var[1]);
}

LinearRegularizedCoulombFriction::LinearRegularizedCoulombFriction() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.01"),velocityUnits(),0));
  var.push_back(new ExtWidget("Marginal velocity",new ExtPhysicalVarWidget(input),true));

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
  var.push_back(new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input)));

  layout->addWidget(var[0]);
  layout->addWidget(var[1]);
}

Function1ChoiceWidget::Function1ChoiceWidget(bool withFactor) : function(0), factor(0) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  if(withFactor) {
    vector<PhysicalStringWidget*> input;
    input.push_back(new PhysicalStringWidget(new ScalarWidget("1"),noUnitUnits(),1));
    factor = new ExtWidget("Factor",new ExtPhysicalVarWidget(input));
    layout->addWidget(factor);
  }

  comboBox = new QComboBox;
  comboBox->addItem(tr("Constant function"));
  comboBox->addItem(tr("Quadratic function"));
  comboBox->addItem(tr("Sinus function"));
  comboBox->addItem(tr("Tabular function"));
  comboBox->addItem(tr("Summation function"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void Function1ChoiceWidget::defineForceLaw(int index) {
  layout->removeWidget(function);
  delete function;
  if(index==0)
    function = new ConstantFunction1("VS");  
  else if(index==1)
    function = new QuadraticFunction1;
  else if(index==2)
    function = new SinusFunction1;
  else if(index==3)
    function = new TabularFunction1;
  else if(index==4) {
    function = new SummationFunction1;
    connect(function,SIGNAL(resize()),this,SIGNAL(resize()));
  }
  layout->addWidget(function);
  emit functionChanged();
  emit resize();
}

Function2ChoiceWidget::Function2ChoiceWidget() : function(0) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Linear spring damper force"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void Function2ChoiceWidget::defineForceLaw(int index) {
  int cols = 0;
  layout->removeWidget(function);
  delete function;
//  if(index==0)
//    function = 0;
  if(index==0) {
    function = new LinearSpringDamperForce;  
    layout->addWidget(function);
  } 
  if(function) {
    //emit resize();
    //connect(function,SIGNAL(resize()),this,SIGNAL(resize()));
  }
}
