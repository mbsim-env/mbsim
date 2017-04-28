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
#include "function.h"
#include <iostream>

using namespace std;

namespace MBSimGUI {

  FunctionWidgetFactory2::FunctionWidgetFactory2(Element *parent_) : parent(parent_) {
    name.push_back("Constant function");
    name.push_back("Linear function");
    name.push_back("Quadratic function");
    name.push_back("Polynom function");
    name.push_back("Sinusoidal function");
    name.push_back("Absolute value function");
    name.push_back("Vector valued function");
    name.push_back("Piecewise defined function");
    name.push_back("Composite function");
    name.push_back("Symbolic function");
    name.push_back("Tabular function");
    name.push_back("Piecewise polynom function");
    name.push_back("Signum function");
    name.push_back("Modulo function");
    name.push_back("Fourier function");
    name.push_back("Signal function");
    name.push_back("Identity function");
    name.push_back("Bidirectional function");
    name.push_back("Continued function");
    xmlName.push_back(MBSIM%"ConstantFunction");
    xmlName.push_back(MBSIM%"LinearFunction");
    xmlName.push_back(MBSIM%"QuadraticFunction");
    xmlName.push_back(MBSIM%"PolynomFunction");
    xmlName.push_back(MBSIM%"SinusoidalFunction");
    xmlName.push_back(MBSIM%"AbsoluteValueFunction");
    xmlName.push_back(MBSIM%"VectorValuedFunction");
    xmlName.push_back(MBSIM%"PiecewiseDefinedFunction");
    xmlName.push_back(MBSIM%"CompositeFunction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"TabularFunction");
    xmlName.push_back(MBSIM%"PiecewisePolynomFunction");
    xmlName.push_back(MBSIM%"SignumFunction");
    xmlName.push_back(MBSIM%"ModuloFunction");
    xmlName.push_back(MBSIM%"FourierFunction");
    xmlName.push_back(MBSIMCONTROL%"SignalFunction");
    xmlName.push_back(MBSIM%"IdentityFunction");
    xmlName.push_back(MBSIM%"BidirectionalFunction");
    xmlName.push_back(MBSIM%"ContinuedFunction");
  }

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
    if(i==8) {
      Function *dummy = new Function("NoName"); // Workaround for correct XML path. TODO: provide a consistent concept
      dummy->setParent(parent);
      return new CompositeFunctionWidget(new FunctionWidgetFactory2(dummy), new FunctionWidgetFactory2(dummy));
    }
    if(i==9)
      return new SymbolicFunctionWidget(QStringList("x"),1,3);
    if(i==10)
      return new TabularFunctionWidget(1);
    if(i==11)
      return new PiecewisePolynomFunctionWidget(1);
    if(i==12)
      return new SignumFunctionWidget(1);
    if(i==13)
      return new ModuloFunctionWidget(1);
    if(i==14)
      return new FourierFunctionWidget(1);
    if(i==15)
      return new SignalFunctionWidget(parent);
    if(i==16)
      return new IdentityFunctionWidget(1);
    if(i==17)
      return new BidirectionalFunctionWidget();
    if(i==18)
      return new ContinuedFunctionWidget(new FunctionWidgetFactory2(parent),new FunctionWidgetFactory2(parent));
    return NULL;
  }

  TranslationWidgetFactory2::TranslationWidgetFactory2(Element *parent_) : parent(parent_) {
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
    name.push_back("Composite function");
    name.push_back("Piecewise polynom function");
    name.push_back("Piecewise defined function");
    xmlName.push_back(MBSIM%"TranslationAlongXAxis");
    xmlName.push_back(MBSIM%"TranslationAlongYAxis");
    xmlName.push_back(MBSIM%"TranslationAlongZAxis");
    xmlName.push_back(MBSIM%"TranslationAlongAxesXY");
    xmlName.push_back(MBSIM%"TranslationAlongAxesYZ");
    xmlName.push_back(MBSIM%"TranslationAlongAxesXZ");
    xmlName.push_back(MBSIM%"TranslationAlongAxesXYZ");
    xmlName.push_back(MBSIM%"TranslationAlongFixedAxis");
    xmlName.push_back(MBSIM%"LinearTranslation");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"CompositeFunction");
    xmlName.push_back(MBSIM%"PiecewisePolynomFunction");
    xmlName.push_back(MBSIM%"PiecewiseDefinedFunction");
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
      return new CompositeFunctionWidget(new TranslationWidgetFactory2(parent), new SymbolicFunctionWidgetFactory1(QStringList("q"),parent));
    if(i==11)
      return new PiecewisePolynomFunctionWidget(3);
    if(i==12)
      return new PiecewiseDefinedFunctionWidget(parent);
    return NULL;
  }

  TranslationWidgetFactory3::TranslationWidgetFactory3(Element *parent_) {
    name.push_back("Vector valued function");
    name.push_back("Composite function");
    name.push_back("Symbolic function");
    name.push_back("Tabular function");
    name.push_back("Piecewise defined function");
    name.push_back("Piecewise polynom function");
    xmlName.push_back(MBSIM%"VectorValuedFunction");
    xmlName.push_back(MBSIM%"CompositeFunction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"TabularFunction");
    xmlName.push_back(MBSIM%"PiecewiseDefinedFunction");
    xmlName.push_back(MBSIM%"PiecewisePolynomFunction");
  }

  QWidget* TranslationWidgetFactory3::createWidget(int i) {
    if(i==0)
      return new VectorValuedFunctionWidget(parent,1,true);
    if(i==1)
      return new CompositeFunctionWidget(new TranslationWidgetFactory2(parent), new FunctionWidgetFactory2(parent));
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

  RotationWidgetFactory2::RotationWidgetFactory2(Element *parent_) : parent(parent_) {
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
    name.push_back("Composite function");
    name.push_back("Symbolic function");
    xmlName.push_back(MBSIM%"RotationAboutXAxis");
    xmlName.push_back(MBSIM%"RotationAboutYAxis");
    xmlName.push_back(MBSIM%"RotationAboutZAxis");
    xmlName.push_back(MBSIM%"RotationAboutAxesXY");
    xmlName.push_back(MBSIM%"RotationAboutAxesYZ");
    xmlName.push_back(MBSIM%"RotationAboutAxesXZ");
    xmlName.push_back(MBSIM%"RotationAboutAxesXYZ");
    xmlName.push_back(MBSIM%"RotationAboutAxesZXZ");
    xmlName.push_back(MBSIM%"RotationAboutAxesZYX");
    xmlName.push_back(MBSIM%"RotationAboutFixedAxis");
    xmlName.push_back(MBSIM%"CompositeFunction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
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
      return new CompositeFunctionWidget(new RotationWidgetFactory2(parent), new SymbolicFunctionWidgetFactory1(QStringList("q"),parent));
    if(i==11)
      return new SymbolicFunctionWidget(QStringList("q"),1,3);
    return NULL;
  }

  RotationWidgetFactory3::RotationWidgetFactory3(Element *parent_) {
    name.push_back("Composite function");
    name.push_back("Symbolic function");
    xmlName.push_back(MBSIM%"CompositeFunction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
  }

  QWidget* RotationWidgetFactory3::createWidget(int i) {
    if(i==0)
      return new CompositeFunctionWidget(new RotationWidgetFactory2(parent), new FunctionWidgetFactory2(parent));
    if(i==1)
      return new SymbolicFunctionWidget(QStringList("t"),1,3);
    return NULL;
  }

  SymbolicFunctionWidgetFactory1::SymbolicFunctionWidgetFactory1(const QStringList &var_, Element *parent_) : parent(parent_), var(var_)  {
    name.push_back("Symbolic function");
    name.push_back("Piecewise polynom function");
    name.push_back("Piecewise defined function");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"PiecewisePolynomFunction");
    xmlName.push_back(MBSIM%"PiecewiseDefinedFunction");
  }

  QWidget* SymbolicFunctionWidgetFactory1::createWidget(int i) {
    if(i==0)
      return new SymbolicFunctionWidget(var,1,3);
    if(i==1)
      return new PiecewisePolynomFunctionWidget(1);
    if(i==2)
      return new PiecewiseDefinedFunctionWidget(parent);
    return NULL;
  }

  SymbolicFunctionWidgetFactory2::SymbolicFunctionWidgetFactory2(const QStringList &var_, Element *parent_) : parent(parent_), var(var_)  {
    name.push_back("Symbolic function");
    name.push_back("Two dimensional tabular function");
    name.push_back("Two dimensional piecewise polynom function");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"TwoDimensionalTabularFunction");
    xmlName.push_back(MBSIM%"TwoDimensionalPiecewisePolynomFunction");
  }

  QWidget* SymbolicFunctionWidgetFactory2::createWidget(int i) {
    if(i==0)
      return new SymbolicFunctionWidget(var,1,100);
    if(i==1)
      return new TwoDimensionalTabularFunctionWidget(1);
    if(i==2)
      return new TwoDimensionalPiecewisePolynomFunctionWidget(1);
    return NULL;
  }

  SymbolicFunctionWidgetFactory3::SymbolicFunctionWidgetFactory3(const QStringList &var_) : var(var_) {
    name.push_back("Symbolic function");
    name.push_back("Modulo function");
    name.push_back("Bounded function");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"ModuloFunction");
    xmlName.push_back(MBSIM%"BoundedFunction");
  }

  QWidget* SymbolicFunctionWidgetFactory3::createWidget(int i) {
    if(i==0)
      return new SymbolicFunctionWidget(var,1,100);
    if(i==1)
      return new ModuloFunctionWidget;
    if(i==2)
      return new BoundedFunctionWidget;
    return NULL;
  }

  TranslationWidgetFactory4::TranslationWidgetFactory4(Element *parent_, const MBXMLUtils::NamespaceURI &uri) : parent(parent_) {
    name.push_back("State dependent translation");
    name.push_back("Time dependent translation");
    name.push_back("General translation");
    xmlName.push_back(uri%"stateDependentTranslation");
    xmlName.push_back(uri%"timeDependentTranslation");
    xmlName.push_back(uri%"generalTranslation");
  }

  QWidget* TranslationWidgetFactory4::createWidget(int i) {
    if(i==0)
      return new ChoiceWidget2(new TranslationWidgetFactory2(parent),QBoxLayout::TopToBottom,0);
    if(i==1)
      return new ChoiceWidget2(new TranslationWidgetFactory3(parent),QBoxLayout::TopToBottom,0);
    if(i==2) {
      QStringList var;
      var << "q" << "t";
      return new ChoiceWidget2(new SymbolicFunctionWidgetFactory2(var,parent),QBoxLayout::TopToBottom,0);
    }
    return NULL;
  }

  RotationWidgetFactory4::RotationWidgetFactory4(Element *parent_, const MBXMLUtils::NamespaceURI &uri) : parent(parent_) {
    name.push_back("State dependent rotation");
    name.push_back("Time dependent rotation");
    xmlName.push_back(uri%"stateDependentRotation");
    xmlName.push_back(uri%"timeDependentRotation");
  }

  QWidget* RotationWidgetFactory4::createWidget(int i) {
    if(i==0)
      return new ChoiceWidget2(new RotationWidgetFactory2(parent),QBoxLayout::TopToBottom,0);
    if(i==1)
      return new ChoiceWidget2(new RotationWidgetFactory3(parent),QBoxLayout::TopToBottom,0);
    return NULL;
  }

  TabularFunctionWidgetFactory::TabularFunctionWidgetFactory() {
    name.push_back("x and y");
    name.push_back("xy");
    xmlName.push_back(MBSIM%"x");
    xmlName.push_back(MBSIM%"xy");
  }

  QWidget* TabularFunctionWidgetFactory::createWidget(int i) {
    if(i==0) {
      ContainerWidget *widgetContainer = new ContainerWidget;
      widgetContainer->addWidget(new ExtWidget("x",new ChoiceWidget2(new VecSizeVarWidgetFactory(3),QBoxLayout::RightToLeft,5),false,false,MBSIM%"x"));
      widgetContainer->addWidget(new ExtWidget("y",new ChoiceWidget2(new MatWidgetFactory(getEye<QString>(3,1,"0","0")),QBoxLayout::RightToLeft,5),false,false,MBSIM%"y"));
      return widgetContainer;
    }
    if(i==1)
      return new ExtWidget("xy",new ChoiceWidget2(new MatRowsVarWidgetFactory(3,2),QBoxLayout::RightToLeft,5),false,false,MBSIM%"xy");
    return NULL;
  }

  TwoDimensionalTabularFunctionWidgetFactory::TwoDimensionalTabularFunctionWidgetFactory() {
    name.push_back("x,y and z");
    name.push_back("xyz");
    xmlName.push_back(MBSIM%"x");
    xmlName.push_back(MBSIM%"xyz");
  }

  QWidget* TwoDimensionalTabularFunctionWidgetFactory::createWidget(int i) {
    if(i==0) {
      ContainerWidget *widgetContainer = new ContainerWidget;
      widgetContainer->addWidget(new ExtWidget("x",new ChoiceWidget2(new VecSizeVarWidgetFactory(3),QBoxLayout::RightToLeft,5),false,false,MBSIM%"x"));
      widgetContainer->addWidget(new ExtWidget("y",new ChoiceWidget2(new VecSizeVarWidgetFactory(3),QBoxLayout::RightToLeft,5),false,false,MBSIM%"y"));
      widgetContainer->addWidget(new ExtWidget("z",new ChoiceWidget2(new MatWidgetFactory(getEye<QString>(3,3,"0","0")),QBoxLayout::RightToLeft,5),false,false,MBSIM%"z"));
      return widgetContainer;
    }
    if(i==1)
      return new ExtWidget("xyz",new ChoiceWidget2(new MatRowsColsVarWidgetFactory(3,3),QBoxLayout::RightToLeft,5),false,false,MBSIM%"xyz");
    return NULL;
  }

  FourierFunctionWidgetFactory::FourierFunctionWidgetFactory() {
    name.push_back("a and b");
    name.push_back("ab");
    xmlName.push_back(MBSIM%"a");
    xmlName.push_back(MBSIM%"ab");
  }

  QWidget* FourierFunctionWidgetFactory::createWidget(int i) {
    if(i==0) {
      ContainerWidget *widgetContainer = new ContainerWidget;
      widgetContainer->addWidget(new ExtWidget("a",new ChoiceWidget2(new VecSizeVarWidgetFactory(3),QBoxLayout::RightToLeft,5),false,false,MBSIM%"a"));
      widgetContainer->addWidget(new ExtWidget("b",new ChoiceWidget2(new VecWidgetFactory(3),QBoxLayout::RightToLeft,5),false,false,MBSIM%"b"));
      return widgetContainer;
    }
    if(i==1)
      return new ExtWidget("ab",new ChoiceWidget2(new MatRowsVarWidgetFactory(3,2),QBoxLayout::RightToLeft,5),false,false,MBSIM%"ab");
    return NULL;
  }

  ConstraintWidgetFactory::ConstraintWidgetFactory(Element *parent_) : parent(parent_) {
    name.push_back("Time dependent constraint function");
    name.push_back("State dependent constraint function");
    xmlName.push_back(MBSIM%"timeDependentConstraintFunction");
    xmlName.push_back(MBSIM%"stateDependentConstraintFunction");
  }

  QWidget* ConstraintWidgetFactory::createWidget(int i) {
    if(i==0)
      return new ExtWidget("Function",new ChoiceWidget2(new FunctionWidgetFactory2(parent),QBoxLayout::TopToBottom,0),false,false,xmlName[i]);
    if(i==1)
      return new ExtWidget("Function",new ChoiceWidget2(new SymbolicFunctionWidgetFactory2(QStringList("q"),parent),QBoxLayout::TopToBottom,0),false,false,xmlName[i]);
    return NULL;
  }

  ConnectRigidBodiesWidgetFactory::ConnectRigidBodiesWidgetFactory(Element *parent_) : parent(parent_) {
    name.push_back("1 rigid body");
    name.push_back("2 rigid bodies");
  }

  QWidget* ConnectRigidBodiesWidgetFactory::createWidget(int i) {
    return new ConnectRigidBodiesWidget(i+1,parent);
  }

  SpringDamperWidgetFactory::SpringDamperWidgetFactory(Element *parent_) : parent(parent_){
    name.push_back("Linear spring damper force");
    name.push_back("Nonlinear spring damper force");
    name.push_back("Symbolic function");
    name.push_back("Linear elastic function");
    xmlName.push_back(MBSIM%"LinearSpringDamperForce");
    xmlName.push_back(MBSIM%"NonlinearSpringDamperForce");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"LinearElasticFunction");
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
    if(i==3)
      return new LinearElasticFunctionWidget;
    return NULL;
  }

  PlanarContourFunctionWidgetFactory::PlanarContourFunctionWidgetFactory(Element *parent_) : parent(parent_){
    name.push_back("Polar contour function");
    name.push_back("Symbolic function");
    name.push_back("Continued function");
    name.push_back("Piecewise polynom function");
    name.push_back("Piecewise defined function");
    name.push_back("Composite function");
    xmlName.push_back(MBSIM%"PolarContourFunction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"ContinuedFunction");
    xmlName.push_back(MBSIM%"PiecewisePolynomFunction");
    xmlName.push_back(MBSIM%"PiecewiseDefinedFunction");
    xmlName.push_back(MBSIM%"CompositeFunction");
  }

  QWidget* PlanarContourFunctionWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PolarContourFunctionWidget;
    if(i==1)
      return new SymbolicFunctionWidget(QStringList("eta"),3,1);
    if(i==2)
      return new ContinuedFunctionWidget(new PlanarContourFunctionWidgetFactory(parent), new SymbolicFunctionWidgetFactory3(QStringList("x")));
    if(i==3)
      return new PiecewisePolynomFunctionWidget(1);
    if(i==4)
      return new PiecewiseDefinedFunctionWidget(parent);
    if(i==5)
      return new CompositeFunctionWidget(new PlanarContourFunctionWidgetFactory(parent), new FunctionWidgetFactory2(parent));
    return NULL;
  }

  SpatialContourFunctionWidgetFactory::SpatialContourFunctionWidgetFactory(Element *parent_) : parent(parent_){
    name.push_back("Symbolic function");
    name.push_back("Continued function");
    name.push_back("Composite function");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"ContinuedFunction");
    xmlName.push_back(MBSIM%"CompositeFunction");
  }

  QWidget* SpatialContourFunctionWidgetFactory::createWidget(int i) {
    if(i==0)
      return new SymbolicFunctionWidget(QStringList("zeta"),3,2);
    if(i==1)
      return new ContinuedFunctionWidget(new SpatialContourFunctionWidgetFactory(parent), new SymbolicFunctionWidgetFactory3(QStringList("x")));
    if(i==2)
      return new CompositeFunctionWidget(new SpatialContourFunctionWidgetFactory(parent), new SymbolicFunctionWidgetFactory3(QStringList("x")));
    return NULL;
  }

}
