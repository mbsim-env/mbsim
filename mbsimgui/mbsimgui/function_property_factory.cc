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
#include "function_property_factory.h"
#include "function_properties.h"
#include "basic_properties.h"
#include "kinematic_functions_properties.h"
#include <vector>

using namespace std;
using namespace MBXMLUtils;

namespace MBSimGUI {

  //Property* FunctionPropertyFactory::createProperty(int i) {
  //  return 0;
  //}
  //
  //Property* TranslationPropertyFactory::createProperty(int i) {
  //  return 0;
  //}
  //
  //Property* RotationPropertyFactory::createProperty(int i) {
  //  return 0;
  //}

  PropertyInterface* FunctionPropertyFactory2::createProperty(int i) {
    if(i==0)
      return new ConstantFunction("NoName",parent);
    if(i==1)
      return new LinearFunction("NoName",parent);
    if(i==2)
      return new QuadraticFunction("NoName",parent);
    if(i==3)
      return new PolynomFunction("NoName",parent);
    if(i==4)
      return new SinusoidalFunction("NoName",parent);
    if(i==5)
      return new AbsoluteValueFunction("NoName",parent);
    if(i==6)
      return new VectorValuedFunction("NoName",parent);
    if(i==7)
      return new PiecewiseDefinedFunction("NoName",parent);
    if(i==8)
      return new NestedFunction("NoName",parent,new FunctionPropertyFactory2(parent),new FunctionPropertyFactory2(parent));
    if(i==9) {
      vector<string> var;
      var.push_back("x");
      var.push_back("y");
      return new BinaryNestedFunction("NoName",parent,new SymbolicFunctionPropertyFactory2(parent,"VVV",var),new FunctionPropertyFactory2(parent),new FunctionPropertyFactory2(parent));
    }
    if(i==10)
      return new SymbolicFunction("NoName",parent,"VS",vector<string>(1,"x"),1);
    if(i==11)
      return new TabularFunction("NoName",parent);
    if(i==12)
      return new PiecewisePolynomFunction("NoName",parent);
    if(i==13)
      return new SignumFunction("NoName",parent);
    if(i==14)
      return new ModuloFunction("NoName",parent);
    if(i==15)
      return new FourierFunction("NoName",parent);
    if(i==16)
      return new SignalFunction("NoName",parent);
    if(i==17)
      return new IdentityFunction("NoName",parent);
    if(i==18)
      return new BidirectionalFunction("NoName",parent);
    if(i==19)
      return new ContinuedFunction("NoName",parent,new FunctionPropertyFactory2(parent),new FunctionPropertyFactory2(parent));
    return NULL;
  }

  vector<FQN> FunctionPropertyFactory2::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"ConstantFunction");
    name.push_back(MBSIM%"LinearFunction");
    name.push_back(MBSIM%"QuadraticFunction");
    name.push_back(MBSIM%"PolynomFunction");
    name.push_back(MBSIM%"SinusoidalFunction");
    name.push_back(MBSIM%"AbsoluteValueFunction");
    name.push_back(MBSIM%"VectorValuedFunction");
    name.push_back(MBSIM%"PiecewiseDefinedFunction");
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"BinaryNestedFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"TabularFunction");
    name.push_back(MBSIM%"PiecewisePolynomFunction");
    name.push_back(MBSIM%"SignumFunction");
    name.push_back(MBSIM%"ModuloFunction");
    name.push_back(MBSIM%"FourierFunction");
    name.push_back(MBSIMCONTROL%"SignalFunction");
    name.push_back(MBSIM%"IdentityFunction");
    name.push_back(MBSIM%"BidirectionalFunction");
    name.push_back(MBSIM%"ContinuedFunction");
    return name;
  }

  PropertyInterface* TranslationPropertyFactory2::createProperty(int i) {
    if(i==0)
      return new TranslationAlongXAxis("NoName",parent);
    if(i==1)
      return new TranslationAlongYAxis("NoName",parent);
    if(i==2)
      return new TranslationAlongZAxis("NoName",parent);
    if(i==3)
      return new TranslationAlongAxesXY("NoName",parent);
    if(i==4)
      return new TranslationAlongAxesYZ("NoName",parent);
    if(i==5)
      return new TranslationAlongAxesXZ("NoName",parent);
    if(i==6)
      return new TranslationAlongAxesXYZ("NoName",parent);
    if(i==7)
      return new TranslationAlongFixedAxis("NoName",parent);
    if(i==8)
      return new LinearTranslation("NoName",parent,3,1);
    if(i==9)
      return new SymbolicFunction("NoName",parent,"VV",vector<string>(1,"q"),3);
    if(i==10)
      return new NestedFunction("NoName",parent,new TranslationPropertyFactory2(parent),new SymbolicFunctionPropertyFactory1(parent,"VV",vector<string>(1,"q")));
    if(i==11)
      return new PiecewisePolynomFunction("NoName",parent);
    if(i==12)
      return new PiecewiseDefinedFunction("NoName",parent);
    return NULL;
  }

  vector<FQN> TranslationPropertyFactory2::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"TranslationAlongXAxis");
    name.push_back(MBSIM%"TranslationAlongYAxis");
    name.push_back(MBSIM%"TranslationAlongZAxis");
    name.push_back(MBSIM%"TranslationAlongAxesXY");
    name.push_back(MBSIM%"TranslationAlongAxesYZ");
    name.push_back(MBSIM%"TranslationAlongAxesXZ");
    name.push_back(MBSIM%"TranslationAlongAxesXYZ");
    name.push_back(MBSIM%"TranslationAlongFixedAxis");
    name.push_back(MBSIM%"LinearTranslation");
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"PiecewisePolynomFunction");
    name.push_back(MBSIM%"PiecewiseDefinedFunction");
    return name;
  }

  PropertyInterface* TranslationPropertyFactory3::createProperty(int i) {
    if(i==0)
      return new VectorValuedFunction("NoName",parent);
    if(i==1)
      return new NestedFunction("NoName",parent,new TranslationPropertyFactory2(parent),new FunctionPropertyFactory2(parent));
    if(i==2) {
      vector<string> var;
      var.push_back("x");
      var.push_back("y");
      return new BinaryNestedFunction("NoName",parent,new SymbolicFunctionPropertyFactory2(parent,"VVV",var),new FunctionPropertyFactory2(parent),new FunctionPropertyFactory2(parent));
    }
    if(i==3)
      return new SymbolicFunction("NoName",parent,"VS",vector<string>(1,"t"),3);
    if(i==4)
      return new TabularFunction("NoName",parent);
    if(i==5)
      return new PiecewiseDefinedFunction("NoName",parent);
    if(i==6)
      return new PiecewisePolynomFunction("NoName",parent);
    return NULL;
  }

  vector<FQN> TranslationPropertyFactory3::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"VectorValuedFunction");
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"BinaryNestedFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"TabularFunction");
    name.push_back(MBSIM%"PiecewiseDefinedFunction");
    name.push_back(MBSIM%"PiecewisePolynomFunction");
    return name;
  }

  PropertyInterface* RotationPropertyFactory2::createProperty(int i) {

    if(i==0)
      return new RotationAboutXAxis("NoName",parent);
    if(i==1)
      return new RotationAboutYAxis("NoName",parent);
    if(i==2)
      return new RotationAboutZAxis("NoName",parent);
    if(i==3)
      return new RotationAboutAxesXY("NoName",parent);
    if(i==4)
      return new RotationAboutAxesYZ("NoName",parent);
    if(i==5)
      return new RotationAboutAxesXZ("NoName",parent);
    if(i==6)
      return new RotationAboutAxesXYZ("NoName",parent);
    if(i==7)
      return new RotationAboutAxesZXZ("NoName",parent);
    if(i==8)
      return new RotationAboutAxesZYX("NoName",parent);
    if(i==9)
      return new RotationAboutFixedAxis("NoName",parent);
    if(i==10)
      return new NestedFunction("NoName",parent,new RotationPropertyFactory2(parent),new SymbolicFunctionPropertyFactory1(parent,"MV",vector<string>(1,"q")));
    if(i==11)
      return new SymbolicFunction("NoName",parent,"MV",vector<string>(1,"q"),1);
    return NULL;
  }

  vector<FQN> RotationPropertyFactory2::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"RotationAboutXAxis");
    name.push_back(MBSIM%"RotationAboutYAxis");
    name.push_back(MBSIM%"RotationAboutZAxis");
    name.push_back(MBSIM%"RotationAboutAxesXY");
    name.push_back(MBSIM%"RotationAboutAxesYZ");
    name.push_back(MBSIM%"RotationAboutAxesXZ");
    name.push_back(MBSIM%"RotationAboutAxesXYZ");
    name.push_back(MBSIM%"RotationAboutAxesZXZ");
    name.push_back(MBSIM%"RotationAboutAxesZYX");
    name.push_back(MBSIM%"RotationAboutFixedAxis");
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    return name;
  }

  PropertyInterface* RotationPropertyFactory3::createProperty(int i) {
    if(i==0)
      return new NestedFunction("NoName",parent,new RotationPropertyFactory2(parent),new FunctionPropertyFactory2(parent));
    if(i==1)
      return new SymbolicFunction("NoName",parent,"MS",vector<string>(1,"t"),1);
    return NULL;
  }

  vector<FQN> RotationPropertyFactory3::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    return name;
  }

  PropertyInterface* SymbolicFunctionPropertyFactory1::createProperty(int i) {
    if(i==0)
      return new SymbolicFunction("NoName",parent,ext,var,1);
    if(i==1)
      return new PiecewisePolynomFunction("NoName",parent);
    if(i==2)
      return new PiecewiseDefinedFunction("NoName",parent);
    return NULL;
  }

  vector<FQN> SymbolicFunctionPropertyFactory1::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"PiecewisePolynomFunction");
    name.push_back(MBSIM%"PiecewiseDefinedFunction");
    return name;
  }

  PropertyInterface* SymbolicFunctionPropertyFactory2::createProperty(int i) {
    if(i==0)
      return new SymbolicFunction("NoName",parent,ext,var,1);
    if(i==1)
      return new TwoDimensionalTabularFunction("NoName",parent);
    if(i==2)
      return new TwoDimensionalPiecewisePolynomFunction("NoName",parent);
    return NULL;
  }

  vector<FQN> SymbolicFunctionPropertyFactory2::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"TwoDimensionalTabularFunction");
    name.push_back(MBSIM%"TwoDimensionalPiecewisePolynomFunction");
    return name;
  }

  PropertyInterface* SymbolicFunctionPropertyFactory3::createProperty(int i) {
    if(i==0)
      return new SymbolicFunction("NoName",parent,ext,var,1);
    if(i==1)
      return new ModuloFunction("NoName",parent);
    if(i==2)
      return new BoundedFunction("NoName",parent);
    return NULL;
  }

  vector<FQN> SymbolicFunctionPropertyFactory3::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"ModuloFunction");
    name.push_back(MBSIM%"BoundedFunction");
    return name;
  }

  TranslationPropertyFactory4::TranslationPropertyFactory4(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"stateDependentTranslation");
    name.push_back(MBSIM%"timeDependentTranslation");
    name.push_back(MBSIM%"generalTranslation");
  }

  PropertyInterface* TranslationPropertyFactory4::createProperty(int i) {
    if(i==0)
      return new ExtProperty(new ChoiceProperty2(new TranslationPropertyFactory2(parent),MBSIM%"stateDependentTranslation"));
    if(i==1)
      return new ExtProperty(new ChoiceProperty2(new TranslationPropertyFactory3(parent),MBSIM%"timeDependentTranslation"));
    if(i==2) {
      vector<string> var;
      var.push_back("q");
      var.push_back("t");
      return new ExtProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory2(parent,"VVS",var),MBSIM%"generalTranslation"));
    }
    return NULL;
  }

  RotationPropertyFactory4::RotationPropertyFactory4(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"stateDependentRotation");
    name.push_back(MBSIM%"timeDependentRotation");
  }

  PropertyInterface* RotationPropertyFactory4::createProperty(int i) {
    if(i==0)
      return new ExtProperty(new ChoiceProperty2(new RotationPropertyFactory2(parent),MBSIM%"stateDependentRotation"));
    if(i==1)
      return new ExtProperty(new ChoiceProperty2(new RotationPropertyFactory3(parent),MBSIM%"timeDependentRotation"));
    return NULL;
  }

  TabularFunctionPropertyFactory::TabularFunctionPropertyFactory(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"x");
    name.push_back(MBSIM%"xy");
  }

  PropertyInterface* TabularFunctionPropertyFactory::createProperty(int i) {
    if(i==0) {
      ContainerProperty *propertyContainer = new ContainerProperty;
      vector<Property*> choiceProperty;

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"x",vector<string>(3,"")),"",4)));

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,1,"1","0"),MBSIM%"y",vector<string>(3,"")),"",4)));

      return propertyContainer;
    }
    if(i==1) {
      return new ExtProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,2,"1","0"),MBSIM%"xy",vector<string>(3,"")),"",4));
    }
    return NULL;
  }

  TwoDimensionalTabularFunctionPropertyFactory::TwoDimensionalTabularFunctionPropertyFactory(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"x");
    name.push_back(MBSIM%"xyz");
  }

  PropertyInterface* TwoDimensionalTabularFunctionPropertyFactory::createProperty(int i) {
    if(i==0) {
      ContainerProperty *propertyContainer = new ContainerProperty;
      vector<Property*> choiceProperty;

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"x",vector<string>(3,"")),"",4)));

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"y",vector<string>(3,"")),"",4)));

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new MatPropertyFactory(getScalars<string>(3,3,"0"),MBSIM%"z",vector<string>(3,"")),"",4)));

      return propertyContainer;
    }
    if(i==1) {
      return new ExtProperty(new ChoiceProperty2(new MatPropertyFactory(getScalars<string>(4,4,"0"),MBSIM%"xyz",vector<string>(3,"")),"",4));
    }
    return NULL;
  }

  FourierFunctionPropertyFactory::FourierFunctionPropertyFactory(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"a");
    name.push_back(MBSIM%"ab");
  }

  PropertyInterface* FourierFunctionPropertyFactory::createProperty(int i) {
    if(i==0) {
      ContainerProperty *propertyContainer = new ContainerProperty;
      vector<Property*> choiceProperty;

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"a",vector<string>(3,"")),"",4)));

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"b",vector<string>(3,"")),"",4)));

      return propertyContainer;
    }
    if(i==1) {
      return new ExtProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,2,"1","0"),MBSIM%"ab",vector<string>(3,"")),"",4));
    }
    return NULL;
  }

  ConstraintPropertyFactory::ConstraintPropertyFactory(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"timeDependentConstraintFunction");
    name.push_back(MBSIM%"stateDependentConstraintFunction");
  }

  PropertyInterface* ConstraintPropertyFactory::createProperty(int i) {
    if(i==0)
      return new ExtProperty(new ChoiceProperty2(new FunctionPropertyFactory2(parent),MBSIM%"timeDependentConstraintFunction"));
    if(i==1)
      return new ExtProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory2(parent,"VV",vector<string>(1,"q")),MBSIM%"stateDependentConstraintFunction"));
    return NULL;
  }

  ConnectFramesPropertyFactory::ConnectFramesPropertyFactory(Element *parent_) : parent(parent_), name(2) {
  }

  PropertyInterface* ConnectFramesPropertyFactory::createProperty(int i) {
    return new ConnectFramesProperty(i+1,parent);
  }

  SpringDamperPropertyFactory::SpringDamperPropertyFactory(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"LinearSpringDamperForce");
    name.push_back(MBSIM%"NonlinearSpringDamperForce");
    name.push_back(MBSIM%"SymbolicFunction");
  }

  PropertyInterface* SpringDamperPropertyFactory::createProperty(int i) {
    if(i==0)
      return new LinearSpringDamperForce("NoName",parent);
    if(i==1)
      return new NonlinearSpringDamperForce("NoName",parent);
    if(i==2) {
      vector<string> var;
      var.push_back("gd");
      var.push_back("g");
      return new SymbolicFunction("NoName",parent,"SSS",var,1);
    }
    return NULL;
  }

  PlanarContourFunctionPropertyFactory::PlanarContourFunctionPropertyFactory(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"PolarContourFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"ContinuedFunction");
    name.push_back(MBSIM%"PiecewisePolynomFunction");
    name.push_back(MBSIM%"PiecewiseDefinedFunction");
    name.push_back(MBSIM%"NestedFunction");
  }

  PropertyInterface* PlanarContourFunctionPropertyFactory::createProperty(int i) {
    if(i==0)
      return new PolarContourFunction("NoName",parent);
    if(i==1)
      return new SymbolicFunction("NoName",parent,"VS",vector<string>(1,"eta"),3);
    if(i==2)
      return new ContinuedFunction("NoName",parent,new PlanarContourFunctionPropertyFactory(parent),new SymbolicFunctionPropertyFactory3(parent,"SS",vector<string>(1,"x")));
    if(i==3)
      return new PiecewisePolynomFunction("NoName",parent);
    if(i==4)
      return new PiecewiseDefinedFunction("NoName",parent);
    if(i==5)
      return new NestedFunction("NoName",parent,new PlanarContourFunctionPropertyFactory(parent),new FunctionPropertyFactory2(parent));
    return NULL;
  }

  SpatialContourFunctionPropertyFactory::SpatialContourFunctionPropertyFactory(Element *parent_) : parent(parent_) {
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"ContinuedFunction");
    name.push_back(MBSIM%"NestedFunction");
  }

  PropertyInterface* SpatialContourFunctionPropertyFactory::createProperty(int i) {
    if(i==0)
      return new SymbolicFunction("NoName",parent,"VV",vector<string>(2,"zeta"),3);
    if(i==1)
      return new ContinuedFunction("NoName",parent,new SpatialContourFunctionPropertyFactory(parent),new SymbolicFunctionPropertyFactory3(parent,"VV",vector<string>(2,"x")));
    if(i==2)
      return new NestedFunction("NoName",parent,new SpatialContourFunctionPropertyFactory(parent),new SymbolicFunctionPropertyFactory3(parent,"VV",vector<string>(2,"x")));
    return NULL;
  }

}
