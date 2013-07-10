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
using namespace fmatvec;
#endif

namespace MBSim {

#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP


  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double)>, SymbolicFunction<double(double)>, MBSIMNS"SymbolicFunction1_SS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(double)>, SymbolicFunction<Vec(double)>, MBSIMNS"SymbolicFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(double)>, SymbolicFunction<VecV(double)>, MBSIMNS"SymbolicFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(double)>, SymbolicFunction<Vec3(double)>, MBSIMNS"SymbolicFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(Vec)>, SymbolicFunction<Vec(Vec)>, MBSIMNS"SymbolicFunction1_VV")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(VecV)>, SymbolicFunction<Vec(VecV)>, MBSIMNS"SymbolicFunction1_VV")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(Vec3)>, SymbolicFunction<Vec(Vec3)>, MBSIMNS"SymbolicFunction1_VV")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(Vec)>, SymbolicFunction<VecV(Vec)>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(VecV)>, SymbolicFunction<VecV(VecV)>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(Vec3)>, SymbolicFunction<VecV(Vec3)>, MBSIMNS"SymbolicFunction1_VV")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(Vec)>, SymbolicFunction<Vec3(Vec)>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(VecV)>, SymbolicFunction<Vec3(VecV)>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(Vec3)>, SymbolicFunction<Vec3(Vec3)>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(Vec)>, SymbolicFunction<double(Vec)>, MBSIMNS"SymbolicFunction1_SV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(VecV)>, SymbolicFunction<double(VecV)>, MBSIMNS"SymbolicFunction1_SV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(Vec3)>, SymbolicFunction<double(Vec3)>, MBSIMNS"SymbolicFunction1_SV")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<double(double,double)>, SymbolicFunction<double(double,double)>, MBSIMNS"SymbolicFunction2_SSS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(Vec,double)>, SymbolicFunction<Vec(Vec,double)>, MBSIMNS"SymbolicFunction2_VVS")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(VecV,double)>, SymbolicFunction<Vec(VecV,double)>, MBSIMNS"SymbolicFunction2_VVS")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(Vec3,double)>, SymbolicFunction<Vec(Vec3,double)>, MBSIMNS"SymbolicFunction2_VVS")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(Vec,double)>, SymbolicFunction<VecV(Vec,double)>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(VecV,double)>, SymbolicFunction<VecV(VecV,double)>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<VecV(Vec3,double)>, SymbolicFunction<VecV(Vec3,double)>, MBSIMNS"SymbolicFunction2_VVS")
  //MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(Vec,double)>, SymbolicFunction<Vec3(Vec,double)>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(VecV,double)>, SymbolicFunction<Vec3(VecV,double)>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec3(Vec3,double)>, SymbolicFunction<Vec3(Vec3,double)>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function<Vec(Vec,Vec)>, SymbolicFunction<Vec(Vec,Vec)>, MBSIMNS"SymbolicFunction2_VVV")

#endif

}
