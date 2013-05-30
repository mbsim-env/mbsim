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

#include <mbsim/utils/symbolic_function.h>

namespace MBSim {

#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<double COMMA double>, MBSIMNS"SymbolicFunction1_SS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec  COMMA double>, MBSIMNS"SymbolicFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<VecV COMMA double>, MBSIMNS"SymbolicFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec3 COMMA double>, MBSIMNS"SymbolicFunction1_VS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec  COMMA Vec >, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec  COMMA VecV>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec  COMMA Vec3>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<VecV COMMA Vec >, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<VecV COMMA VecV>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<VecV COMMA Vec3>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec3 COMMA Vec >, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec3 COMMA VecV>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec3 COMMA Vec3>, MBSIMNS"SymbolicFunction1_VV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<double COMMA Vec >, MBSIMNS"SymbolicFunction1_SV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<double COMMA VecV>, MBSIMNS"SymbolicFunction1_SV")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<double COMMA Vec3>, MBSIMNS"SymbolicFunction1_SV")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<double  COMMA double  COMMA double>, MBSIMNS"SymbolicFunction2_SSS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec  COMMA Vec  COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec  COMMA VecV COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec  COMMA Vec3 COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<VecV COMMA Vec  COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<VecV COMMA VecV COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<VecV COMMA Vec3 COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec3 COMMA Vec  COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec3 COMMA VecV COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec3 COMMA Vec3 COMMA double>, MBSIMNS"SymbolicFunction2_VVS")

#endif

}
