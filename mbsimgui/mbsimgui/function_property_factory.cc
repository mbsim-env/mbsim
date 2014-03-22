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

  Property* FunctionPropertyFactory2::createProperty(int i) {
    if(i==0)
      return new ConstantFunctionProperty;
    if(i==1)
      return new LinearFunctionProperty;
    if(i==2)
      return new QuadraticFunctionProperty;
    if(i==3)
      return new PolynomFunctionProperty;
    if(i==4)
      return new SinusoidalFunctionProperty;
    if(i==5)
      return new AbsoluteValueFunctionProperty;
    if(i==6)
      return new PointSymmetricFunctionProperty;
    if(i==7)
      return new LineSymmetricFunctionProperty;
    if(i==8)
      return new ScaledFunctionProperty;
    if(i==9)
      return new SummationFunctionProperty;
    if(i==10)
      return new VectorValuedFunctionProperty;
    if(i==11)
      return new PiecewiseDefinedFunctionProperty;
    if(i==12)
      return new NestedFunctionProperty(new FunctionPropertyFactory2, new FunctionPropertyFactory2);
    if(i==13)
      return new SymbolicFunctionProperty("VS",vector<string>(1,"t"),1);
    if(i==14)
      return new TabularFunctionProperty;
  }

  vector<FQN> FunctionPropertyFactory2::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"ConstantFunction");
    name.push_back(MBSIM%"LinearFunction");
    name.push_back(MBSIM%"QuadraticFunction");
    name.push_back(MBSIM%"PolynomFunction");
    name.push_back(MBSIM%"SinusoidalFunction");
    name.push_back(MBSIM%"AbsoluteValueFunction");
    name.push_back(MBSIM%"PointSymmetricFunction");
    name.push_back(MBSIM%"LineSymmetricFunction");
    name.push_back(MBSIM%"ScaledFunction");
    name.push_back(MBSIM%"SummationFunction");
    name.push_back(MBSIM%"VectorValuedFunction");
    name.push_back(MBSIM%"PiecewiseDefinedFunction");
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"TabularFunction");
    return name;
  }

  Property* TranslationPropertyFactory2::createProperty(int i) {
    if(i==0)
      return new TranslationAlongXAxisProperty;
    if(i==1)
      return new TranslationAlongYAxisProperty;
    if(i==2)
      return new TranslationAlongZAxisProperty;
    if(i==3)
      return new TranslationAlongAxesXYProperty;
    if(i==4)
      return new TranslationAlongAxesYZProperty;
    if(i==5)
      return new TranslationAlongAxesXZProperty;
    if(i==6)
      return new TranslationAlongAxesXYZProperty;
    if(i==7)
      return new TranslationAlongFixedAxisProperty;
    if(i==8)
      return new LinearTranslationProperty(3,1);
    if(i==9)
      return new SymbolicFunctionProperty("VV",vector<string>(1,"q"),3);
    if(i==10)
      return new NestedFunctionProperty(new TranslationPropertyFactory2, new SymbolicFunctionPropertyFactory2("VV",vector<string>(1,"q")));
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
    return name;
  }

  Property* TranslationPropertyFactory3::createProperty(int i) {
    if(i==0)
      return new VectorValuedFunctionProperty;
    if(i==1)
      return new NestedFunctionProperty(new TranslationPropertyFactory2, new FunctionPropertyFactory2);
    if(i==2)
      return new SymbolicFunctionProperty("VS",vector<string>(1,"t"),3);
    if(i==3)
      return new TabularFunctionProperty;
    if(i==4)
      return new ScaledFunctionProperty;
    if(i==5)
      return new SummationFunctionProperty;
    if(i==6)
      return new PiecewiseDefinedFunctionProperty;
  }

  vector<FQN> TranslationPropertyFactory3::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"VectorValuedFunction");
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"TabularFunction");
    name.push_back(MBSIM%"ScaledFunction");
    name.push_back(MBSIM%"SummationFunction");
    name.push_back(MBSIM%"PiecewiseDefinedFunction");
    return name;
  }

  Property* RotationPropertyFactory2::createProperty(int i) {

    if(i==0)
      return new RotationAboutXAxisProperty;
    if(i==1)
      return new RotationAboutYAxisProperty;
    if(i==2)
      return new RotationAboutZAxisProperty;
    if(i==3)
      return new RotationAboutAxesXYProperty;
    if(i==4)
      return new RotationAboutAxesYZProperty;
    if(i==5)
      return new RotationAboutAxesXZProperty;
    if(i==6)
      return new RotationAboutAxesXYZProperty;
    if(i==7)
      return new RotationAboutFixedAxisProperty;
    if(i==8)
      return new NestedFunctionProperty(new RotationPropertyFactory2, new SymbolicFunctionPropertyFactory2("MV",vector<string>(1,"q")));
    if(i==9)
      return new SymbolicFunctionProperty("MV",vector<string>(1,"q"),1);
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
    name.push_back(MBSIM%"RotationAboutFixedAxis");
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    return name;
  }

  Property* RotationPropertyFactory3::createProperty(int i) {
    if(i==0)
      return new NestedFunctionProperty(new RotationPropertyFactory2, new FunctionPropertyFactory2);
    if(i==1)
      return new SymbolicFunctionProperty("MS",vector<string>(1,"t"),1);
  }

  vector<FQN> RotationPropertyFactory3::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"NestedFunction");
    name.push_back(MBSIM%"SymbolicFunction");
    return name;
  }

  Property* SymbolicFunctionPropertyFactory2::createProperty(int i) {
    if(i==0)
      return new SymbolicFunctionProperty(ext,var,1);
    if(i==1)
      return new TwoDimensionalTabularFunctionProperty;
  }

  vector<FQN> SymbolicFunctionPropertyFactory2::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"TwoDimensionalTabularFunction");
    return name;
  }

  Property* SymbolicFunctionPropertyFactory3::createProperty(int i) {
    if(i==0)
      return new SymbolicFunctionProperty(ext,var,1);
    if(i==1)
      return new ModuloFunctionProperty;
  }

  vector<FQN> SymbolicFunctionPropertyFactory3::getNames() {
    vector<FQN> name;
    name.push_back(MBSIM%"SymbolicFunction");
    name.push_back(MBSIM%"ModuloFunction");
    return name;
  }

  TranslationPropertyFactory4::TranslationPropertyFactory4() {
    name.push_back(MBSIM%"stateDependentTranslation");
    name.push_back(MBSIM%"timeDependentTranslation");
    name.push_back(MBSIM%"generalTranslation");
  }

  Property* TranslationPropertyFactory4::createProperty(int i) {
    if(i==0)
      return new ExtProperty(new ChoiceProperty2(new TranslationPropertyFactory2,MBSIM%"stateDependentTranslation"));
    if(i==1)
      return new ExtProperty(new ChoiceProperty2(new TranslationPropertyFactory3,MBSIM%"timeDependentTranslation"));
    if(i==2) {
      vector<string> var;
      var.push_back("q");
      var.push_back("t");
      return new ExtProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory2("VVS",var),MBSIM%"generalTranslation"));
    }
  }

  RotationPropertyFactory4::RotationPropertyFactory4() {
    name.push_back(MBSIM%"stateDependentRotation");
    name.push_back(MBSIM%"timeDependentRotation");
  }

  Property* RotationPropertyFactory4::createProperty(int i) {
    if(i==0)
      return new ExtProperty(new ChoiceProperty2(new RotationPropertyFactory2,MBSIM%"stateDependentRotation"));
    if(i==1)
      return new ExtProperty(new ChoiceProperty2(new RotationPropertyFactory3,MBSIM%"timeDependentRotation"));
  }

  TabularFunctionPropertyFactory::TabularFunctionPropertyFactory() {
    name.push_back(MBSIM%"x");
    name.push_back(MBSIM%"xy");
  }

  Property* TabularFunctionPropertyFactory::createProperty(int i) {
    if(i==0) {
      ContainerProperty *propertyContainer = new ContainerProperty;
      vector<Property*> choiceProperty;

      //    vector<PhysicalVariableProperty> input;
      //    input.push_back(PhysicalVariableProperty(new VecFromFileProperty,"",MBSIM%"x"));
      //    propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new VecPropertyFactory(3,MBSIM%"x",vector<string>(3,"")),"",4)));

      propertyContainer->addProperty(new ExtProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,1,"1","0"),MBSIM%"y",vector<string>(3,"")),"",4)));
      //    input.clear();
      //    input.push_back(PhysicalVariableProperty(new MatFromFileProperty,"",MBSIM%"y"));
      //    propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

      return propertyContainer;
    }
    if(i==1) {
      //vector<PhysicalVariableProperty> input;
      //input.push_back(PhysicalVariableProperty(new MatFromFileProperty,"",MBSIM%"xy"));
      //return new ExtProperty(new ExtPhysicalVarProperty(input));
      return new ExtProperty(new ChoiceProperty2(new MatPropertyFactory(getEye<string>(3,2,"1","0"),MBSIM%"xy",vector<string>(3,"")),"",4));
    }
  }

  ConstraintPropertyFactory::ConstraintPropertyFactory() {
    name.push_back(MBSIM%"timeDependentConstraintFunction");
    name.push_back(MBSIM%"stateDependentConstraintFunction");
  }

  Property* ConstraintPropertyFactory::createProperty(int i) {
    if(i==0)
      return new ExtProperty(new ChoiceProperty2(new FunctionPropertyFactory2,MBSIM%"timeDependentConstraintFunction"));
    if(i==1)
      return new ExtProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory2("VV",vector<string>(1,"q")),MBSIM%"stateDependentConstraintFunction"));
  }

  ConnectFramesPropertyFactory::ConnectFramesPropertyFactory(Element *element_) : name(2), element(element_) {
  }

  Property* ConnectFramesPropertyFactory::createProperty(int i) {
    return new ConnectFramesProperty(i+1,element);
  }

  SpringDamperPropertyFactory::SpringDamperPropertyFactory() {
    name.push_back(MBSIM%"LinearSpringDamperForce");
    name.push_back(MBSIM%"NonlinearSpringDamperForce");
    name.push_back(MBSIM%"SymbolicFunction");
  }

  Property* SpringDamperPropertyFactory::createProperty(int i) {
    if(i==0)
      return new LinearSpringDamperForceProperty;
    if(i==1)
      return new NonlinearSpringDamperForceProperty;
    if(i==2) {
      vector<string> var;
      var.push_back("gd");
      var.push_back("g");
      return new SymbolicFunctionProperty("SSS",var,1);
    }
  }

}
