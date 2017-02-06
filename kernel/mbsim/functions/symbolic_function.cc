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
#include <mbsim/functions/symbolic_function.h>
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double(double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3(double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV(double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec(double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double(Vec3)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double(VecV)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double(Vec)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3(Vec3)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3(VecV)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV(Vec3)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV(VecV)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec(Vec)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3(Vec2)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2(Vec2)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(VecV)>)

  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double(double,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3(double,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3(Vec3,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3(VecV,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV(Vec3,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV(VecV,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec(Vec,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV(VecV,VecV)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec(Vec,Vec)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(VecV,double)>)

}
