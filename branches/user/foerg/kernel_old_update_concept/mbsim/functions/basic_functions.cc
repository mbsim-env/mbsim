/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */


#include <config.h>
#include "mbsim/functions/basic_functions.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {


  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ConstantFunction<double(double)>, MBSIM%"ConstantFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ConstantFunction<VecV(double)>, MBSIM%"ConstantFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ConstantFunction<Vec3(double)>, MBSIM%"ConstantFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ConstantFunction<VecV(VecV)>, MBSIM%"ConstantFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(LinearFunction<double(double)>, MBSIM%"LinearFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(LinearFunction<VecV(double)>, MBSIM%"LinearFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(LinearFunction<Vec3(double)>, MBSIM%"LinearFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(LinearFunction<VecV(VecV)>, MBSIM%"LinearFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(QuadraticFunction<double(double)>, MBSIM%"QuadraticFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(QuadraticFunction<VecV(double)>, MBSIM%"QuadraticFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(QuadraticFunction<Vec3(double)>, MBSIM%"QuadraticFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(QuadraticFunction<VecV(VecV)>, MBSIM%"QuadraticFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(PolynomFunction<double(double)>, MBSIM%"PolynomFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(PolynomFunction<VecV(double)>, MBSIM%"PolynomFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(PolynomFunction<Vec3(double)>, MBSIM%"PolynomFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(PolynomFunction<VecV(VecV)>, MBSIM%"PolynomFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SinusoidalFunction<double(double)>, MBSIM%"SinusoidalFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SinusoidalFunction<VecV(double)>, MBSIM%"SinusoidalFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SinusoidalFunction<Vec3(double)>, MBSIM%"SinusoidalFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SinusoidalFunction<VecV(VecV)>, MBSIM%"SinusoidalFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(StepFunction<double(double)>, MBSIM%"StepFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(StepFunction<VecV(double)>, MBSIM%"StepFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(StepFunction<VecV(VecV)>, MBSIM%"StepFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(AbsoluteValueFunction<double(double)>, MBSIM%"AbsoluteValueFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(AbsoluteValueFunction<VecV(double)>, MBSIM%"AbsoluteValueFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(AbsoluteValueFunction<VecV(VecV)>, MBSIM%"AbsoluteValueFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ModuloFunction<double(double)>, MBSIM%"ModuloFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ModuloFunction<VecV(double)>, MBSIM%"ModuloFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ModuloFunction<VecV(VecV)>, MBSIM%"ModuloFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ModuloFunction<Vec(Vec)>, MBSIM%"ModuloFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SignumFunction<double(double)>, MBSIM%"SignumFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SignumFunction<VecV(double)>, MBSIM%"SignumFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SignumFunction<VecV(VecV)>, MBSIM%"SignumFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(BoundedFunction<double(double)>, MBSIM%"PositiveFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(BoundedFunction<VecV(double)>, MBSIM%"PositiveFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(BoundedFunction<VecV(VecV)>, MBSIM%"PositiveFunction")
}
