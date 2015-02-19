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
#include "function_widget_factory.h"
#include "function_widgets.h"
#include "kinematic_functions_widgets.h"
#include "extended_widgets.h"
#include "variable_widgets.h"
#include "basic_widgets.h"
#include <iostream>

using namespace std;

namespace MBSimGUI {

  QWidget* FunctionWidgetFactory2::createWidget(int i) {
    if(i==0)
      return new ConstantFunctionWidget(1); 
    if(i==1)
      return new LinearFunctionWidget(1); 
    if(i==2)
      return new QuadraticFunctionWidget(1); 
    if(i==3)
      return new PolynomFunctionWidget(1);
    if(i==4)
      return new SinusoidalFunctionWidget(1);
    if(i==5)
      return new AbsoluteValueFunctionWidget(1);
    if(i==6)
      return new VectorValuedFunctionWidget(parent,1,true);
    if(i==7)
      return new PiecewiseDefinedFunctionWidget(parent);
    if(i==8)
      return new NestedFunctionWidget(new FunctionWidgetFactory2(parent), new FunctionWidgetFactory2(parent));
    if(i==9)
      return new SymbolicFunctionWidget(QStringList("t"),1,3);
    if(i==10)
      return new TabularFunctionWidget(1);
    if(i==11)
      return new PiecewisePolynomFunctionWidget(1);
    if(i==12)
      return new SignumFunctionWidget(1);
    if(i==13)
      return new AdditionFunctionWidget(parent,1);
    if(i==14)
      return new MultiplicationFunctionWidget(parent,1);
    if(i==15)
      return new FourierFunctionWidget(1);
    if(i==16)
      return new SignalFunctionWidget(parent);
    return NULL;
  }

  vector<QString> FunctionWidgetFactory2::getNames() {
    vector<QString> name;
    name.push_back("Constant function");
    name.push_back("Linear function");
    name.push_back("Quadratic function");
    name.push_back("Polynom function");
    name.push_back("Sinusoidal function");
    name.push_back("Absolute value function");
    name.push_back("Vector valued function");
    name.push_back("Piecewise defined function");
    name.push_back("Nested function");
    name.push_back("Symbolic function");
    name.push_back("Tabular function");
    name.push_back("Piecewise polynom function");
    name.push_back("Signum function");
    name.push_back("Addition function");
    name.push_back("Multiplication function");
    name.push_back("Fourier function");
    name.push_back("Signal function");
    return name;
  }

  QWidget* TranslationWidgetFactory2::createWidget(int i) {
    if(i==0)
      return new TranslationAlongXAxisWidget;
    if(i==1)
      return new TranslationAlongYAxisWidget;
    if(i==2)
      return new TranslationAlongZAxisWidget;
    if(i==3)
      return new TranslationAlongAxesXYWidget;
    if(i==4)
      return new TranslationAlongAxesYZWidget;
    if(i==5)
      return new TranslationAlongAxesXZWidget;
    if(i==6)
      return new TranslationAlongAxesXYZWidget;
    if(i==7)
      return new TranslationAlongFixedAxisWidget;
    if(i==8)
      return new LinearTranslationWidget(3,1);
    if(i==9)
      return new SymbolicFunctionWidget(QStringList("q"),3,3);
    if(i==10)
      return new NestedFunctionWidget(new TranslationWidgetFactory2(parent), new SymbolicFunctionWidgetFactory2(QStringList("q")));
    return NULL;
  }

  vector<QString> TranslationWidgetFactory2::getNames() {
    vector<QString> name;
    name.push_back("Translation along x axis");
    name.push_back("Translation along y axis");
    name.push_back("Translation along z axis");
    name.push_back("Translation along axes x and y");
    name.push_back("Translation along axes y and z");
    name.push_back("Translation along axes x and z");
    name.push_back("Translation along axes x,y and z");
    name.push_back("Translation along fixed axis");
    name.push_back("Linear translation");
    name.push_back("Symbolic function");
    name.push_back("Nested function");
    return name;
  }

  QWidget* TranslationWidgetFactory3::createWidget(int i) {
    if(i==0)
      return new VectorValuedFunctionWidget(parent,1,true);
    if(i==1)
      return new NestedFunctionWidget(new TranslationWidgetFactory2(parent), new FunctionWidgetFactory2(parent));
    if(i==2)
      return new SymbolicFunctionWidget(QStringList("t"),3,3);
    if(i==3)
      return new TabularFunctionWidget(1);
    if(i==4)
      return new PiecewiseDefinedFunctionWidget(parent);
    if(i==5)
      return new PiecewisePolynomFunctionWidget(1);
    return NULL;
  }

  vector<QString> TranslationWidgetFactory3::getNames() {
    vector<QString> name;
    name.push_back("Vector valued function");
    name.push_back("Nested function");
    name.push_back("Symbolic function");
    name.push_back("Tabular function");
    name.push_back("Piecewise defined function");
    name.push_back("Piecewise polynom function");
    return name;
  }

  QWidget* RotationWidgetFactory2::createWidget(int i) {

    if(i==0)
      return new RotationAboutXAxisWidget;
    if(i==1)
      return new RotationAboutYAxisWidget;
    if(i==2)
      return new RotationAboutZAxisWidget;
    if(i==3)
      return new RotationAboutAxesXYWidget;
    if(i==4)
      return new RotationAboutAxesYZWidget;
    if(i==5)
      return new RotationAboutAxesXZWidget;
    if(i==6)
      return new RotationAboutAxesXYZWidget;
    if(i==7)
      return new RotationAboutAxesZXZWidget;
    if(i==8)
      return new RotationAboutAxesZYXWidget;
    if(i==9)
      return new RotationAboutFixedAxisWidget;
    if(i==10)
      return new NestedFunctionWidget(new RotationWidgetFactory2(parent), new SymbolicFunctionWidgetFactory2(QStringList("q")));
    if(i==11)
      return new SymbolicFunctionWidget(QStringList("q"),1,3);
    return NULL;
  }

  vector<QString> RotationWidgetFactory2::getNames() {
    vector<QString> name;
    name.push_back("Rotation about x axis");
    name.push_back("Rotation about y axis");
    name.push_back("Rotation about z axis");
    name.push_back("Rotation about axes x and y");
    name.push_back("Rotation about axes y and z");
    name.push_back("Rotation about axes x and z");
    name.push_back("Rotation about axes x,y and z");
    name.push_back("Rotation about axes z,x and z");
    name.push_back("Rotation about axes z,y and x");
    name.push_back("Rotation about fixed axis");
    name.push_back("Nested function");
    name.push_back("Symbolic function");
    return name;
  }

  QWidget* RotationWidgetFactory3::createWidget(int i) {
    if(i==0)
      return new NestedFunctionWidget(new RotationWidgetFactory2(parent), new FunctionWidgetFactory2(parent));
    if(i==1)
      return new SymbolicFunctionWidget(QStringList("t"),1,3);
    return NULL;
  }

  vector<QString> RotationWidgetFactory3::getNames() {
    vector<QString> name;
    name.push_back("Nested function");
    name.push_back("Symbolic function");
    return name;
  }

  QWidget* SymbolicFunctionWidgetFactory2::createWidget(int i) {
    if(i==0)
      return new SymbolicFunctionWidget(var,1,3);
    if(i==1)
      return new TwoDimensionalTabularFunctionWidget(1);
    return NULL;
  }

  vector<QString> SymbolicFunctionWidgetFactory2::getNames() {
    vector<QString> name;
    name.push_back("Symbolic function");
    name.push_back("Two dimensional tabular function");
    return name;
  }
  QWidget* SymbolicFunctionWidgetFactory3::createWidget(int i) {
    if(i==0)
      return new SymbolicFunctionWidget(var,1,3);
    if(i==1)
      return new ModuloFunctionWidget;
    return NULL;
  }

  vector<QString> SymbolicFunctionWidgetFactory3::getNames() {
    vector<QString> name;
    name.push_back("Symbolic function");
    name.push_back("Modulo function");
    return name;
  }

  TranslationWidgetFactory4::TranslationWidgetFactory4(Element *parent_) : parent(parent_) {
    name.push_back("State dependent translation");
    name.push_back("Time dependent translation");
    name.push_back("General translation");
  }

  QWidget* TranslationWidgetFactory4::createWidget(int i) {
    if(i==0)
      return new ExtWidget("Function r=r(q)",new ChoiceWidget2(new TranslationWidgetFactory2(parent)));
    if(i==1)
      return new ExtWidget("Function r=r(t)",new ChoiceWidget2(new TranslationWidgetFactory3(parent)));
    if(i==2) {
      QStringList var;
      var << "q" << "t";
      return new ExtWidget("Function r=r(q,t)",new ChoiceWidget2(new SymbolicFunctionWidgetFactory2(var)));
    }
    return NULL;
  }

  RotationWidgetFactory4::RotationWidgetFactory4(Element *parent_) : parent(parent_) {
    name.push_back("State dependent rotation");
    name.push_back("Time dependent rotation");
  }

  QWidget* RotationWidgetFactory4::createWidget(int i) {
    if(i==0)
      return new ExtWidget("Function r=r(q)",new ChoiceWidget2(new RotationWidgetFactory2(parent)));
    if(i==1)
      return new ExtWidget("Function r=r(t)",new ChoiceWidget2(new RotationWidgetFactory3(parent)));
    return NULL;
  }

  TabularFunctionWidgetFactory::TabularFunctionWidgetFactory() {
    name.push_back("x and y");
    name.push_back("xy");
  }

  QWidget* TabularFunctionWidgetFactory::createWidget(int i) {
    if(i==0) {
      ContainerWidget *widgetContainer = new ContainerWidget;
      widgetContainer->addWidget(new ExtWidget("x",new ChoiceWidget2(new VecSizeVarWidgetFactory(3,vector<QStringList>(3,QStringList())))));
      widgetContainer->addWidget(new ExtWidget("y",new ChoiceWidget2(new MatWidgetFactory(getEye<QString>(3,1,"0","0"),vector<QStringList>(3,QStringList()),vector<int>(3,0)))));
      return widgetContainer;
    }
    if(i==1) {
      return new ExtWidget("xy",new ChoiceWidget2(new MatRowsVarWidgetFactory(getEye<QString>(3,2,"1","0"),vector<QStringList>(3,QStringList()),vector<int>(3,0))));
    }
    return NULL;
  }

  FourierFunctionWidgetFactory::FourierFunctionWidgetFactory() {
    name.push_back("a and b");
    name.push_back("ab");
  }

  QWidget* FourierFunctionWidgetFactory::createWidget(int i) {
    if(i==0) {
      ContainerWidget *widgetContainer = new ContainerWidget;
      widgetContainer->addWidget(new ExtWidget("a",new ChoiceWidget2(new VecSizeVarWidgetFactory(3,vector<QStringList>(3,QStringList())))));
      widgetContainer->addWidget(new ExtWidget("b",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,QStringList())))));
      return widgetContainer;
    }
    if(i==1) {
      return new ExtWidget("ab",new ChoiceWidget2(new MatRowsVarWidgetFactory(getEye<QString>(3,2,"0","0"),vector<QStringList>(3,QStringList()),vector<int>(3,0))));
    }
    return NULL;
  }

  ConstraintWidgetFactory::ConstraintWidgetFactory(Element *parent_) : parent(parent_) {
    name.push_back("Time dependent constraint function");
    name.push_back("State dependent constraint function");
  }

  QWidget* ConstraintWidgetFactory::createWidget(int i) {
    if(i==0)
      return new ExtWidget("Function",new ChoiceWidget2(new FunctionWidgetFactory2(parent)));
    if(i==1)
      return new ExtWidget("Function",new ChoiceWidget2(new SymbolicFunctionWidgetFactory2(QStringList("q"))));
    return NULL;
  }

  ConnectFramesWidgetFactory::ConnectFramesWidgetFactory(Element *parent_) : parent(parent_) {
    name.push_back("1 frame");
    name.push_back("2 frames");
  }

  QWidget* ConnectFramesWidgetFactory::createWidget(int i) {
    return new ConnectFramesWidget(i+1,parent);
  }

  SpringDamperWidgetFactory::SpringDamperWidgetFactory(Element *parent_) : parent(parent_){
    name.push_back("Linear spring damper force");
    name.push_back("Nonlinear spring damper force");
    name.push_back("Symbolic function");
  }

  QWidget* SpringDamperWidgetFactory::createWidget(int i) {
    if(i==0)
      return new LinearSpringDamperForceWidget;
    if(i==1)
      return new NonlinearSpringDamperForceWidget(parent);
    if(i==2) {
      QStringList var;
      var << "g" << "gd";
      return new SymbolicFunctionWidget(var,1,3);
    }
    return NULL;
  }

}
