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

class FunctionWidgetFactory : public WidgetFactory {
  public:
    FunctionWidgetFactory(const QString &ext_, int n_) : ext(ext_), n(n_) { }
    Widget* createWidget();
  protected:
    QString ext;
    int n;
};

Widget* FunctionWidgetFactory::createWidget() {
  QString tmp = ext.mid(0,1);
  vector<QWidget*> widget;
  vector<QString> name;
  widget.push_back(new ConstantFunctionWidget(tmp,n)); name.push_back("Constant function");
  widget.push_back(new LinearFunctionWidget(tmp,n)); name.push_back("Linear function");
  widget.push_back(new QuadraticFunctionWidget(tmp,n)); name.push_back("Quadratic function");
  widget.push_back(new SinusoidalFunctionWidget(tmp,n)); name.push_back("Sinus function");
  widget.push_back(new TabularFunctionWidget(tmp,n)); name.push_back("Tabular function");
  widget.push_back(new LinearCombinationFunctionWidget(ext,n)); name.push_back("LinearCombination function");
  widget.push_back(new PiecewiseDefinedFunctionWidget(tmp,n)); name.push_back("Piecewise defined function");
  widget.push_back(new SymbolicFunctionWidget(ext,QStringList("t"))); name.push_back("Symbolic function");

  return new ChoiceWidget(widget,name);
}

class LinearCombinationWidgetFactory : public WidgetFactory {
  public:
    LinearCombinationWidgetFactory(int n_) : n(n_) { }
    Widget* createWidget();
  protected:
    int n;
};

Widget* LinearCombinationWidgetFactory::createWidget() {
  FunctionWidgetFactory factory("VS",n);

  ContainerWidget *widget = new ContainerWidget;
  widget->addWidget(new ExtWidget("Function",factory.createWidget()));

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),noUnitUnits(),1));
  widget->addWidget(new ExtWidget("Factor",new ExtPhysicalVarWidget(input),true));

  return widget;
}

class PiecewiseDefinedFunctionWidgetFactory : public WidgetFactory {
  public:
    PiecewiseDefinedFunctionWidgetFactory(int n_) : n(n_) { }
    Widget* createWidget();
  protected:
    int n;
};

Widget* PiecewiseDefinedFunctionWidgetFactory::createWidget() {
  FunctionWidgetFactory factory("VS",n);

  ContainerWidget *widget = new ContainerWidget;
  widget->addWidget(new ExtWidget("Function",factory.createWidget()));

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
  widget->addWidget(new ExtWidget("Limit",new ExtPhysicalVarWidget(input)));

  return widget;
}

ConstantFunctionWidget::ConstantFunctionWidget(const QString &ext, int m) : FunctionWidget(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  if(ext[0]=='V') {
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
    c = new ExtWidget("Value (transposed)",new ExtPhysicalVarWidget(input));
  }
  else {
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
    c = new ExtWidget("Value",new ExtPhysicalVarWidget(input));
  }
  layout->addWidget(c);
}

void ConstantFunctionWidget::resize_(int m, int n) {
  if(ext[0]=='V') {
    if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    }
  }
}

LinearFunctionWidget::LinearFunctionWidget(const QString &ext, int m) : FunctionWidget(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"),QStringList(),0));
  a = new ExtWidget("Slope",new ExtPhysicalVarWidget(input));
  layout->addWidget(a);

  input.clear();
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  b = new ExtWidget("Intercept",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(b);
}

void LinearFunctionWidget::resize_(int m, int n) {
  if(ext[0]=='V') {
    if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(b->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    }
  }
}

NestedFunctionWidget::NestedFunctionWidget(const QString &ext, const vector<QWidget*> &widget_, const vector<QString> &name_) : FunctionWidget(ext) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  fo = new ExtWidget("Outer function",new ChoiceWidget(widget_,name_));
  layout->addWidget(fo);
  connect(fo->getWidget(),SIGNAL(widgetChanged()),this,SLOT(resizeVariables()));

  vector<QWidget*> widget;
  vector<QString> name;
  QStringList var;
  if(ext[1]=='V' and ext[2]=='V') {
    var << "q";
    widget.push_back(new SymbolicFunctionWidget("VV",var)); name.push_back("Symbolic function");
  }
  else if(ext[1]=='V' and ext[2]=='S') {
    var << "t";
    widget.push_back(new SymbolicFunctionWidget("VS",var)); name.push_back("Symbolic function");
    widget.push_back(new ConstantFunctionWidget("V")); name.push_back("Constant function");
    widget.push_back(new LinearFunctionWidget("V")); name.push_back("Linear function v(t)");
    widget.push_back(new QuadraticFunctionWidget("V")); name.push_back("Quadratic function");
    widget.push_back(new SinusoidalFunctionWidget("V")); name.push_back("Sinus function");
  }
  else if(ext[1]=='S' and ext[2]=='V') {
    var << "q";
    widget.push_back(new SymbolicFunctionWidget("SV",var)); name.push_back("Symbolic function");
  }
  else if(ext[1]=='S' and ext[2]=='S') {
    var << "t";
    widget.push_back(new SymbolicFunctionWidget("SS",var)); name.push_back("Symbolic function");
    widget.push_back(new ConstantFunctionWidget("S")); name.push_back("Constant function");
    widget.push_back(new LinearFunctionWidget("S")); name.push_back("Linear function");
    widget.push_back(new QuadraticFunctionWidget("S")); name.push_back("Quadratic function");
    widget.push_back(new SinusoidalFunctionWidget("S")); name.push_back("Sinus function");
  }
  fi = new ExtWidget("Inner function",new ChoiceWidget(widget,name));
  layout->addWidget(fi);
  connect((ChoiceWidget*)fi->getWidget(),SIGNAL(resize_()),this,SLOT(resizeVariables()));
}

int NestedFunctionWidget::getArg1Size() const {
  return ext[2]=='V'?static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(fi->getWidget())->getWidget())->getArg1Size():0;
}

void NestedFunctionWidget::resizeVariables() {
  int size = static_cast<FunctionWidget*>(static_cast<ChoiceWidget*>(fo->getWidget())->getWidget())->getArg1Size();
  static_cast<ChoiceWidget*>(fi->getWidget())->resize_(size,1);
}

VectorValuedFunctionWidget::VectorValuedFunctionWidget(int m, bool fixedSize) : FunctionWidget("V") {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  functions = new ListWidget(new FunctionWidgetFactory("SS",1),"Function",m,1,fixedSize);
  layout->addWidget(functions);
}

void VectorValuedFunctionWidget::resize_(int m, int n) {
  if(ext[0]=='V') {
    functions->resize_(m,n);
  }
}

PiecewiseDefinedFunctionWidget::PiecewiseDefinedFunctionWidget(const QString &ext, int n) : FunctionWidget(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),noUnitUnits(),1));
  contDiff = new ExtWidget("Continously differentiable",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(contDiff);

  functions = new ListWidget(new PiecewiseDefinedFunctionWidgetFactory(n),"Function");
  layout->addWidget(functions);
 }

void PiecewiseDefinedFunctionWidget::resize_(int m, int n) {
  if(ext[0]=='V') {
    functions->resize_(m,n);
  }
}

TranslationAlongFixedAxisWidget::TranslationAlongFixedAxisWidget(const QString &ext) : FunctionWidget(ext) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(3),QStringList(),0));
  a = new ExtWidget("Axis of translation",new ExtPhysicalVarWidget(input));
  layout->addWidget(a);
}

LinearTranslationWidget::LinearTranslationWidget(const QString &ext, int m, int n) : FunctionWidget(ext) {
//  MatColsVarWidget* m = new MatColsVarWidget(3,1,1,3);
//  input.push_back(new PhysicalVariableWidget(m,noUnitUnits(),1));
//  ExtPhysicalVarWidget *mat_ = new ExtPhysicalVarWidget(input);
//  mat = new ExtWidget("Translation vectors",mat_);
//  layout->addWidget(mat);
//  QObject::connect(m, SIGNAL(sizeChanged(int)), this, SIGNAL(translationChanged()));
//  QObject::connect(mat_, SIGNAL(inputDialogChanged(int)), this, SIGNAL(translationChanged()));

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  if(ext[0]=='V') {
    MatColsVarWidget *a_ = new MatColsVarWidget(m,1,1,3);
    input.push_back(new PhysicalVariableWidget(a_,QStringList(),0));
    connect(a_,SIGNAL(sizeChanged(int)),this,SIGNAL(arg1SizeChanged(int)));
  }
  else
    input.push_back(new PhysicalVariableWidget(new VecWidget(m),QStringList(),0));
  A = new ExtWidget("Slope",new ExtPhysicalVarWidget(input));
  layout->addWidget(A);

  input.clear();
  input.push_back(new PhysicalVariableWidget(new VecWidget(m),QStringList(),0));
  b = new ExtWidget("Intercept",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(b);
}

int LinearTranslationWidget::getArg1Size() const {
  if(ext[0]=='V') {
    string str = evalOctaveExpression(static_cast<ExtPhysicalVarWidget*>(A->getWidget())->getCurrentPhysicalVariableWidget()->getValue().toStdString());
    vector<vector<string> > A = strToMat(str);
    return A.size()?A[0].size():0;
  }
  return 0;
}

void LinearTranslationWidget::resize_(int m, int n) {
//  if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m)
//    ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(c->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
}

RotationAboutFixedAxisWidget::RotationAboutFixedAxisWidget(const QString &ext) : FunctionWidget(ext) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalVariableWidget*> input;
  input.push_back(new PhysicalVariableWidget(new VecWidget(3),QStringList(),0));
  a = new ExtWidget("Axis of rotation",new ExtPhysicalVarWidget(input));
  layout->addWidget(a);
}

QuadraticFunctionWidget::QuadraticFunctionWidget(const QString &ext, int m) : FunctionWidget(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  a0 = new ExtWidget("a0",new ExtPhysicalVarWidget(input));
  layout->addWidget(a0);

  input.clear();
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  a1 = new ExtWidget("a1",new ExtPhysicalVarWidget(input));
  layout->addWidget(a1);

  input.clear();
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  a2 = new ExtWidget("a2",new ExtPhysicalVarWidget(input));
  layout->addWidget(a2);
}

void QuadraticFunctionWidget::resize_(int m, int n) {
  if(ext[0]=='V') {
    if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a0->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a1->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a2->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    }
  }
}

SinusoidalFunctionWidget::SinusoidalFunctionWidget(const QString &ext, int m) : FunctionWidget(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalVariableWidget*> input;
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  a = new ExtWidget("Amplitude",new ExtPhysicalVarWidget(input));
  layout->addWidget(a);

  input.clear();
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  f = new ExtWidget("Frequency",new ExtPhysicalVarWidget(input));
  layout->addWidget(f);

  input.clear();
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  p = new ExtWidget("Phase",new ExtPhysicalVarWidget(input));
  layout->addWidget(p);

  input.clear();
  if(ext[0]=='V')
    input.push_back(new PhysicalVariableWidget(new VecWidget(m,true),QStringList(),0));
  else
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"),QStringList(),0));
  o = new ExtWidget("Offset",new ExtPhysicalVarWidget(input),true);
  layout->addWidget(o);
}

void SinusoidalFunctionWidget::resize_(int m, int n) {
  if(ext[0]=='V') {
    if(((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a->getWidget())->getPhysicalVariableWidget(0)->getWidget())->size() != m) {
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(a->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(f->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(p->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
      ((VecWidget*)static_cast<ExtPhysicalVarWidget*>(o->getWidget())->getPhysicalVariableWidget(0)->getWidget())->resize_(m);
    }
  }
}

TabularFunctionWidget::TabularFunctionWidget(const QString &ext, int n) : FunctionWidget(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<QWidget*> choiceWidget;
  vector<QString> name;
  name.push_back("x and y");
  name.push_back("xy");
  ContainerWidget *widgetContainer = new ContainerWidget;
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

  choice = new ChoiceWidget(choiceWidget,name);
  layout->addWidget(choice);
}

LinearCombinationFunctionWidget::LinearCombinationFunctionWidget(const QString &ext, int n) : FunctionWidget(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  functions = new ListWidget(new LinearCombinationWidgetFactory(n),"Function");
  layout->addWidget(functions);
 }

void LinearCombinationFunctionWidget::resize_(int m, int n) {
  if(ext[0]=='V') {
    functions->resize_(m,n);
  }
}

SymbolicFunctionWidget::SymbolicFunctionWidget(const QString &ext, const QStringList &var, int max) : FunctionWidget(ext) {
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
  f = new ExtWidget("Function",new OctaveExpressionWidget);
  layout->addWidget(f,var.size(),0,1,2);
}

int SymbolicFunctionWidget::getArg1Size() const {
  return (ext[1]=='V')?static_cast<SpinBoxWidget*>(argdim[0]->getWidget())->getValue():0;
}

int SymbolicFunctionWidget::getArg2Size() const {
  return (ext[2]=='V')?static_cast<SpinBoxWidget*>(argdim[1]->getWidget())->getValue():0;
}

void SymbolicFunctionWidget::setArg1Size(int i) {
  static_cast<SpinBoxWidget*>(argdim[0]->getWidget())->setValue(i);
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
