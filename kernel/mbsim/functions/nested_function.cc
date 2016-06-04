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
#include "mbsim/functions/nested_function.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  // The following functions are created using ...create<Function<Vec3(VecV)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<Vec3   (double(VecV  ))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<Vec3   (VecV  (VecV  ))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<Vec3(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<Vec3   (double(double))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<Vec3   (VecV  (double))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<RotMat3(VecV)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<RotMat3(double(VecV  ))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<RotMat3(VecV  (VecV  ))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<RotMat3(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<RotMat3(double(double))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<RotMat3(VecV  (double))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<double(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<double (double(double))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<double (VecV  (double))>, MBSIM%"NestedFunction")
  // The following functions are created using ...create<Function<VecV(double)> >(...). Hence the "second"
  // template argument is undefined! Hence we define first the one with the less general form (but being the fastest).
  // If it comes to an dimension error during the initialization of this Function we just try it with the next one.
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<VecV   (double(double))>, MBSIM%"NestedFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<VecV   (VecV  (double))>, MBSIM%"NestedFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(NestedFunction<Vec3   (Vec2(Vec2  ))>, MBSIM%"NestedFunction")
}
