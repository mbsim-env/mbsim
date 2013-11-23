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

#include "mbsim/functions/basic_functions.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {


  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantFunction<double>, MBSIMNS"ConstantFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ConstantFunction<VecV>, MBSIMNS"ConstantFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearFunction<double>, MBSIMNS"LinearFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearFunction<VecV>, MBSIMNS"LinearFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, QuadraticFunction<double>, MBSIMNS"QuadraticFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, QuadraticFunction<VecV>, MBSIMNS"QuadraticFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SinusoidalFunction<double>, MBSIMNS"SinusoidalFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SinusoidalFunction<VecV>, MBSIMNS"SinusoidalFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PositiveFunction<double>, MBSIMNS"PositiveFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PositiveFunction<VecV>, MBSIMNS"PositiveFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, StepFunction<double>, MBSIMNS"StepFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, StepFunction<VecV>, MBSIMNS"StepFunction")

  // The following functions are created using ...create<Function<Vec3(VecV)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<Vec3   (double(VecV  ))>, MBSIMNS"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<Vec3   (VecV  (VecV  ))>, MBSIMNS"NestedFunction")
  // The following functions are created using ...create<Function<Vec3(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<Vec3   (double(double))>, MBSIMNS"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<Vec3   (VecV  (double))>, MBSIMNS"NestedFunction")
  // The following functions are created using ...create<Function<RotMat3(VecV)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<RotMat3(double(VecV  ))>, MBSIMNS"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<RotMat3(VecV  (VecV  ))>, MBSIMNS"NestedFunction")
  // The following functions are created using ...create<Function<RotMat3(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<RotMat3(double(double))>, MBSIMNS"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, NestedFunction<RotMat3(VecV  (double))>, MBSIMNS"NestedFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ScaledFunction<VecV>, MBSIMNS"ScaledFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ScaledFunction<Vec3>, MBSIMNS"ScaledFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, ScaledFunction<double>, MBSIMNS"ScaledFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SummationFunction<VecV>, MBSIMNS"SummationFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SummationFunction<Vec3>, MBSIMNS"SummationFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SummationFunction<double>, MBSIMNS"SummationFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, VectorValuedFunction<VecV>, MBSIMNS"VectorValuedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, VectorValuedFunction<Vec3>, MBSIMNS"VectorValuedFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PiecewiseDefinedFunction<VecV>, MBSIMNS"PiecewiseDefinedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PiecewiseDefinedFunction<Vec3>, MBSIMNS"PiecewiseDefinedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PiecewiseDefinedFunction<double>, MBSIMNS"PiecewiseDefinedFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PolynomFunction<VecV>, MBSIMNS"PolynomFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PolynomFunction<Vec3>, MBSIMNS"PolynomFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PolynomFunction<double>, MBSIMNS"PolynomFunction")
}
