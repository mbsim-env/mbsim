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
    return new ScaledFunctionProperty;
  if(i==6)
    return new SummationFunctionProperty;
  if(i==7)
    return new VectorValuedFunctionProperty;
  if(i==8)
    return new PiecewiseDefinedFunctionProperty;
  if(i==9)
    return new SymbolicFunctionProperty("VS",vector<string>(1,"t"));
  if(i==10)
    return new TabularFunctionProperty;
}

vector<string> FunctionPropertyFactory2::getNames() {
  vector<std::string> name;
  name.push_back(MBSIMNS"ConstantFunction");
  name.push_back(MBSIMNS"LinearFunction");
  name.push_back(MBSIMNS"QuadraticFunction");
  name.push_back(MBSIMNS"PolynomFunction");
  name.push_back(MBSIMNS"SinusoidalFunction");
  name.push_back(MBSIMNS"ScaledFunction");
  name.push_back(MBSIMNS"SummationFunction");
  name.push_back(MBSIMNS"VectorValuedFunction");
  name.push_back(MBSIMNS"PiecewiseDefinedFunction");
  name.push_back(MBSIMNS"SymbolicFunction");
  name.push_back(MBSIMNS"TabularFunction");
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
    return new SymbolicFunctionProperty("VV",vector<string>(1,"q"));
  if(i==10)
    return new NestedFunctionProperty(new TranslationPropertyFactory2, new SymbolicFunctionPropertyFactory2("VV",vector<string>(1,"q")));
}

vector<string> TranslationPropertyFactory2::getNames() {
  vector<std::string> name;
  name.push_back(MBSIMNS"TranslationAlongXAxis");
  name.push_back(MBSIMNS"TranslationAlongYAxis");
  name.push_back(MBSIMNS"TranslationAlongZAxis");
  name.push_back(MBSIMNS"TranslationAlongAxesXY");
  name.push_back(MBSIMNS"TranslationAlongAxesYZ");
  name.push_back(MBSIMNS"TranslationAlongAxesXZ");
  name.push_back(MBSIMNS"TranslationAlongAxesXYZ");
  name.push_back(MBSIMNS"TranslationAlongFixedAxis");
  name.push_back(MBSIMNS"LinearTranslation");
  name.push_back(MBSIMNS"SymbolicFunction");
  name.push_back(MBSIMNS"NestedFunction");
  return name;
}

Property* TranslationPropertyFactory3::createProperty(int i) {
  if(i==0)
    return new VectorValuedFunctionProperty;
  if(i==1)
    return new NestedFunctionProperty(new TranslationPropertyFactory2, new FunctionPropertyFactory2);
  if(i==2)
    return new SymbolicFunctionProperty("VS",vector<string>(1,"t"));
  if(i==3)
    return new TabularFunctionProperty;
  if(i==4)
    return new ScaledFunctionProperty;
  if(i==5)
    return new SummationFunctionProperty;
  if(i==6)
    return new PiecewiseDefinedFunctionProperty;
}

vector<string> TranslationPropertyFactory3::getNames() {
  vector<std::string> name;
  name.push_back(MBSIMNS"VectorValuedFunction");
  name.push_back(MBSIMNS"NestedFunction");
  name.push_back(MBSIMNS"SymbolicFunction");
  name.push_back(MBSIMNS"TabularFunction");
  name.push_back(MBSIMNS"ScaledFunction");
  name.push_back(MBSIMNS"SummationFunction");
  name.push_back(MBSIMNS"PiecewiseDefinedFunction");
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
    return new SymbolicFunctionProperty("MV",vector<string>(1,"q"));
}

vector<string> RotationPropertyFactory2::getNames() {
  vector<string> name;
  name.push_back(MBSIMNS"RotationAboutXAxis");
  name.push_back(MBSIMNS"RotationAboutYAxis");
  name.push_back(MBSIMNS"RotationAboutZAxis");
  name.push_back(MBSIMNS"RotationAboutAxesXY");
  name.push_back(MBSIMNS"RotationAboutAxesYZ");
  name.push_back(MBSIMNS"RotationAboutAxesXZ");
  name.push_back(MBSIMNS"RotationAboutAxesXYZ");
  name.push_back(MBSIMNS"RotationAboutFixedAxis");
  name.push_back(MBSIMNS"NestedFunction");
  name.push_back(MBSIMNS"SymbolicFunction");
  return name;
}

Property* RotationPropertyFactory3::createProperty(int i) {
  if(i==0)
    return new NestedFunctionProperty(new RotationPropertyFactory2, new FunctionPropertyFactory2);
  if(i==1)
    return new SymbolicFunctionProperty("MS",vector<string>(1,"t"));
}

vector<string> RotationPropertyFactory3::getNames() {
  vector<std::string> name;
  name.push_back(MBSIMNS"NestedFunction");
  name.push_back(MBSIMNS"SymbolicFunction");
  return name;
}

Property* SymbolicFunctionPropertyFactory2::createProperty(int i) {
  return new SymbolicFunctionProperty(ext,var);
}

vector<string> SymbolicFunctionPropertyFactory2::getNames() {
  vector<std::string> name;
  name.push_back(MBSIMNS"SymbolicFunction");
  return name;
}

TranslationPropertyFactory4::TranslationPropertyFactory4() {
  name.push_back(MBSIMNS"stateDependentTranslation");
  name.push_back(MBSIMNS"timeDependentTranslation");
  name.push_back(MBSIMNS"generalTranslation");
}

Property* TranslationPropertyFactory4::createProperty(int i) {
  if(i==0)
    return new ExtProperty(new ChoiceProperty2(new TranslationPropertyFactory2,MBSIMNS"stateDependentTranslation"));
  if(i==1)
    return new ExtProperty(new ChoiceProperty2(new TranslationPropertyFactory3,MBSIMNS"timeDependentTranslation"));
  if(i==2) {
    vector<string> var;
    var.push_back("q");
    var.push_back("t");
    return new ExtProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory2("VVS",var),MBSIMNS"generalTranslation"));
  }
}

RotationPropertyFactory4::RotationPropertyFactory4() {
  name.push_back(MBSIMNS"stateDependentRotation");
  name.push_back(MBSIMNS"timeDependentRotation");
}

Property* RotationPropertyFactory4::createProperty(int i) {
  if(i==0)
    return new ExtProperty(new ChoiceProperty2(new RotationPropertyFactory2,MBSIMNS"stateDependentRotation"));
  if(i==1)
    return new ExtProperty(new ChoiceProperty2(new RotationPropertyFactory3,MBSIMNS"timeDependentRotation"));
}

TabularFunctionPropertyFactory::TabularFunctionPropertyFactory() {
  name.push_back(MBSIMNS"x");
  name.push_back(MBSIMNS"xy");
}

Property* TabularFunctionPropertyFactory::createProperty(int i) {
  if(i==0) {
    ContainerProperty *propertyContainer = new ContainerProperty;
    vector<Property*> choiceProperty;

    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new VecFromFileProperty,"",MBSIMNS"x"));
    propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

    input.clear();
    input.push_back(PhysicalVariableProperty(new MatFromFileProperty,"",MBSIMNS"y"));
    propertyContainer->addProperty(new ExtProperty(new ExtPhysicalVarProperty(input)));

    return propertyContainer;
  }
  if(i==1) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new MatFromFileProperty,"",MBSIMNS"xy"));
    return new ExtProperty(new ExtPhysicalVarProperty(input));
  }
}

ConstraintPropertyFactory::ConstraintPropertyFactory() {
  name.push_back(MBSIMNS"timeDependentConstraintFunction");
  name.push_back(MBSIMNS"stateDependentConstraintFunction");
}

Property* ConstraintPropertyFactory::createProperty(int i) {
  if(i==0)
    return new ExtProperty(new ChoiceProperty2(new FunctionPropertyFactory2,MBSIMNS"timeDependentConstraintFunction"));
  if(i==1)
    return new ExtProperty(new ChoiceProperty2(new SymbolicFunctionPropertyFactory2("VV",vector<string>(1,"q")),MBSIMNS"stateDependentConstraintFunction"));
}

ConnectFramesPropertyFactory::ConnectFramesPropertyFactory(Element *element_) : name(2), element(element_) {
}

Property* ConnectFramesPropertyFactory::createProperty(int i) {
  return new ConnectFramesProperty(i+1,element);
}

SpringDamperPropertyFactory::SpringDamperPropertyFactory() {
  name.push_back(MBSIMNS"LinearSpringDamperForce");
  name.push_back(MBSIMNS"SymbolicFunction");
}

Property* SpringDamperPropertyFactory::createProperty(int i) {
  if(i==0)
    return new LinearSpringDamperForceProperty;
  if(i==1) {
    vector<string> var;
    var.push_back("gd");
    var.push_back("g");
    return new SymbolicFunctionProperty("SSS",var);
  }
}
