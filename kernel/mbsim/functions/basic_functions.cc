/* Copyright (C) 2004-2009 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: markus.ms.schneider@gmail.com
 */


#include <config.h>
#include "mbsim/functions/basic_functions.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {


  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ConstantFunction<double(double)>, MBSIM%"ConstantFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ConstantFunction<VecV(double)>, MBSIM%"ConstantFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ConstantFunction<Vec3(double)>, MBSIM%"ConstantFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ConstantFunction<VecV(VecV)>, MBSIM%"ConstantFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LinearFunction<double(double)>, MBSIM%"LinearFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LinearFunction<VecV(double)>, MBSIM%"LinearFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LinearFunction<Vec3(double)>, MBSIM%"LinearFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LinearFunction<VecV(VecV)>, MBSIM%"LinearFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(QuadraticFunction<double(double)>, MBSIM%"QuadraticFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(QuadraticFunction<VecV(double)>, MBSIM%"QuadraticFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(QuadraticFunction<Vec3(double)>, MBSIM%"QuadraticFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(QuadraticFunction<VecV(VecV)>, MBSIM%"QuadraticFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PolynomFunction<double(double)>, MBSIM%"PolynomFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PolynomFunction<VecV(double)>, MBSIM%"PolynomFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PolynomFunction<Vec3(double)>, MBSIM%"PolynomFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PolynomFunction<VecV(VecV)>, MBSIM%"PolynomFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SinusoidalFunction<double(double)>, MBSIM%"SinusoidalFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SinusoidalFunction<VecV(double)>, MBSIM%"SinusoidalFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SinusoidalFunction<Vec3(double)>, MBSIM%"SinusoidalFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SinusoidalFunction<VecV(VecV)>, MBSIM%"SinusoidalFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(StepFunction<double(double)>, MBSIM%"StepFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(StepFunction<VecV(double)>, MBSIM%"StepFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(StepFunction<VecV(VecV)>, MBSIM%"StepFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AbsoluteValueFunction<double(double)>, MBSIM%"AbsoluteValueFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AbsoluteValueFunction<VecV(double)>, MBSIM%"AbsoluteValueFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AbsoluteValueFunction<VecV(VecV)>, MBSIM%"AbsoluteValueFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ModuloFunction<double(double)>, MBSIM%"ModuloFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ModuloFunction<VecV(double)>, MBSIM%"ModuloFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ModuloFunction<VecV(VecV)>, MBSIM%"ModuloFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ModuloFunction<Vec(Vec)>, MBSIM%"ModuloFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SignumFunction<double(double)>, MBSIM%"SignumFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SignumFunction<VecV(double)>, MBSIM%"SignumFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SignumFunction<VecV(VecV)>, MBSIM%"SignumFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PositiveFunction<double(double)>, MBSIM%"PositiveFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PositiveFunction<VecV(double)>, MBSIM%"PositiveFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PositiveFunction<VecV(VecV)>, MBSIM%"PositiveFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PointSymmetricFunction<double(double)>, MBSIM%"PointSymmetricFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PointSymmetricFunction<VecV(double)>, MBSIM%"PointSymmetricFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LineSymmetricFunction<double(double)>, MBSIM%"LineSymmetricFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LineSymmetricFunction<VecV(double)>, MBSIM%"LineSymmetricFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ScaledFunction<VecV(double)>, MBSIM%"ScaledFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ScaledFunction<Vec3(double)>, MBSIM%"ScaledFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ScaledFunction<double(double)>, MBSIM%"ScaledFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SummationFunction<VecV(double)>, MBSIM%"SummationFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SummationFunction<Vec3(double)>, MBSIM%"SummationFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SummationFunction<double(double)>, MBSIM%"SummationFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AdditionFunction<VecV(double)>, MBSIM%"AdditionFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AdditionFunction<Vec3(double)>, MBSIM%"AdditionFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AdditionFunction<double(double)>, MBSIM%"AdditionFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(MultiplicationFunction<VecV(double)>, MBSIM%"MultiplicationFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(MultiplicationFunction<Vec3(double)>, MBSIM%"MultiplicationFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(MultiplicationFunction<double(double)>, MBSIM%"MultiplicationFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(VectorValuedFunction<VecV(double)>, MBSIM%"VectorValuedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(VectorValuedFunction<Vec3(double)>, MBSIM%"VectorValuedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(VectorValuedFunction<VecV(VecV)>, MBSIM%"VectorValuedFunction")

  // The following functions are created using ...create<Function<Vec3(VecV)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<Vec3   (double(VecV  ))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<Vec3   (VecV  (VecV  ))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<Vec3(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<Vec3   (double(double))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<Vec3   (VecV  (double))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<RotMat3(VecV)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<RotMat3(double(VecV  ))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<RotMat3(VecV  (VecV  ))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<RotMat3(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<RotMat3(double(double))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<RotMat3(VecV  (double))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<double(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<double(double(double))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<double(VecV  (double))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<VecV(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<VecV(double(double))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(NestedFunction<VecV(VecV(double))>, MBSIM%"NestedFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PiecewiseDefinedFunction<VecV(double)>, MBSIM%"PiecewiseDefinedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PiecewiseDefinedFunction<Vec3(double)>, MBSIM%"PiecewiseDefinedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PiecewiseDefinedFunction<double(double)>, MBSIM%"PiecewiseDefinedFunction")
}
