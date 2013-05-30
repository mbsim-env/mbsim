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

#include "mbsim/utils/ppolynom.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/mbsim_event.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PPolynom<Ref COMMA Ref>, MBSIMNS"PiecewisePolynom1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PPolynom<Var COMMA Var>, MBSIMNS"PiecewisePolynom1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PPolynom<Var COMMA Fixed<3> >, MBSIMNS"PiecewisePolynom1_VS")
      
}

