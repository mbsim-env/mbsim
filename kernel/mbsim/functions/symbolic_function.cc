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

  /* Instantiate all combinations of:
   *
   * A return type: double, Vec3, Vec2, VecV, Vec, RotMat3
   * with one argument of type: double, Vec3, Vec2, VecV, Vec
   *
   * A return type: double, Vec3, Vec2, VecV, Vec, RotMat3
   * with two arguments of argument 1 type: double, Vec3, Vec2, VecV, Vec
   *                   and argument 2 type: double, Vec3, Vec2, VecV, Vec
   *
   * Combinations for which the type of parDer (or parDer1/parDer2) is not defined are skipped (not listed here at all)
   * (e.g. d(VecV)/d(Vec3)=MatVx3=Error is not defined whereas d(Vec3)/d(VecV)=Mat3xV is defined)
   *
   * For the type Vec2 only a few relevant combinations are used (most are commented out)
   * to avoid the instantiation of many functions which will never be used
   *
   * Fixed size vectors other than Vec2 and Vec3 are not used since these are usually not used at all
   *
   * A return type of Mat, MatV, MatVx3, ... is also not used for now due to the lack of usage but may be
   * added later, at least for arguments of type double for which parDer is defined
   */
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (double       )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (double       )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (double       )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (double       )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec    (double       )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(double       )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec3         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec3         )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec3         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec3         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec3         )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec2         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec2         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec2         )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec2         )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec2         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (VecV         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (VecV         )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (VecV         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (VecV         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(VecV         )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec          )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec    (Vec          )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (double,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (double,double)>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (double,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (double,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec    (double,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(double,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec3  ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec3  ,double)>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec3  ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec3  ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec3  ,double)>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec2  ,double)>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec2  ,double)>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec2  ,double)>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec2  ,double)>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec2  ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (VecV  ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (VecV  ,double)>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (VecV  ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (VecV  ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(VecV  ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec   ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec    (Vec   ,double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec3  ,Vec3  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec3  ,Vec3  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec3  ,Vec3  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec3  ,Vec3  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec3  ,Vec3  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec2  ,Vec3  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec2  ,Vec3  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec2  ,Vec3  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec2  ,Vec3  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec2  ,Vec3  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (VecV  ,Vec3  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (VecV  ,Vec3  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (VecV  ,Vec3  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (VecV  ,Vec3  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(VecV  ,Vec3  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec   ,Vec3  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec3  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec3  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec3  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec3  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec3  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec2  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec2  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec2  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec2  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec2  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (VecV  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (VecV  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (VecV  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (VecV  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(VecV  ,Vec2  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec   ,Vec2  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec3  ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec3  ,VecV  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec3  ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec3  ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec3  ,VecV  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec2  ,VecV  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (Vec2  ,VecV  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (Vec2  ,VecV  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (Vec2  ,VecV  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(Vec2  ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (VecV  ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec3   (VecV  ,VecV  )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec2   (VecV  ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<VecV   (VecV  ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<RotMat3(VecV  ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec   ,VecV  )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec3  ,Vec   )>)
//MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec2  ,Vec   )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (VecV  ,Vec   )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<double (Vec   ,Vec   )>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, SymbolicFunction<Vec    (Vec   ,Vec   )>)

}
