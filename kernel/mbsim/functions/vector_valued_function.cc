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
#include "mbsim/functions/vector_valued_function.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, VectorValuedFunction<VecV(double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, VectorValuedFunction<Vec3(double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, VectorValuedFunction<Vec2(double)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, VectorValuedFunction<Vec3(Vec2)>)
  MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(MBSIM, VectorValuedFunction<VecV(VecV)>)

}
