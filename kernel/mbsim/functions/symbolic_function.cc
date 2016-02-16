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
#ifdef HAVE_CASADI
#include <mbsim/functions/symbolic_function.h>
using namespace fmatvec;
#endif

namespace MBSim {

#ifdef HAVE_CASADI

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<double(double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec3(double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<VecV(double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec(double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<double(Vec3)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<double(VecV)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<double(Vec)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec3(Vec3)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec3(VecV)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<VecV(Vec3)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<VecV(VecV)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec(Vec)>, MBSIM%"SymbolicFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<double(double,double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec3(Vec3,double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec3(VecV,double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<VecV(Vec3,double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<VecV(VecV,double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec(Vec,double)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<VecV(VecV,VecV)>, MBSIM%"SymbolicFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(SymbolicFunction<Vec(Vec,Vec)>, MBSIM%"SymbolicFunction")

#endif

}
