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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
#include <mbsim/utils/symbolic_function.h>
#include <mbsim/element.h>
using namespace fmatvec;
#endif

namespace MBSim {

#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP


  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<double(double)>, MBSIMNS"SymbolicFunction_SS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec(double)>, MBSIMNS"SymbolicFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<VecV(double)>, MBSIMNS"SymbolicFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec3(double)>, MBSIMNS"SymbolicFunction_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec(Vec)>, MBSIMNS"SymbolicFunction_VV")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec(VecV)>, MBSIMNS"SymbolicFunction_VV")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec(Vec3)>, MBSIMNS"SymbolicFunction_VV")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<VecV(Vec)>, MBSIMNS"SymbolicFunction_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<VecV(VecV)>, MBSIMNS"SymbolicFunction_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<VecV(Vec3)>, MBSIMNS"SymbolicFunction_VV")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec3(Vec)>, MBSIMNS"SymbolicFunction_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec3(VecV)>, MBSIMNS"SymbolicFunction_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec3(Vec3)>, MBSIMNS"SymbolicFunction_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<double(Vec)>, MBSIMNS"SymbolicFunction_SV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<double(VecV)>, MBSIMNS"SymbolicFunction_SV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<double(Vec3)>, MBSIMNS"SymbolicFunction_SV")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<double(double,double)>, MBSIMNS"SymbolicFunction_SSS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec(Vec,double)>, MBSIMNS"SymbolicFunction_VVS")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec(VecV,double)>, MBSIMNS"SymbolicFunction_VVS")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec(Vec3,double)>, MBSIMNS"SymbolicFunction_VVS")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<VecV(Vec,double)>, MBSIMNS"SymbolicFunction_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<VecV(VecV,double)>, MBSIMNS"SymbolicFunction_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<VecV(Vec3,double)>, MBSIMNS"SymbolicFunction_VVS")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec3(Vec,double)>, MBSIMNS"SymbolicFunction_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec3(VecV,double)>, MBSIMNS"SymbolicFunction_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec3(Vec3,double)>, MBSIMNS"SymbolicFunction_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, SymbolicFunction<Vec(Vec,Vec)>, MBSIMNS"SymbolicFunction_VVV")

#endif

}
