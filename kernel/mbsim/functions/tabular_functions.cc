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
 * Contact: markus.ms.schneider@gmail.com
 */

#include "mbsim/functions/tabular_functions.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TabularFunction<double(double)>, MBSIMNS"TabularFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TabularFunction<VecV(double)>, MBSIMNS"TabularFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TabularFunction<VecV(VecV)>, MBSIMNS"TabularFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PeriodicTabularFunction<double(double)>, MBSIMNS"PeriodicTabularFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PeriodicTabularFunction<VecV(double)>, MBSIMNS"PeriodicTabularFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, PeriodicTabularFunction<VecV(VecV)>, MBSIMNS"PeriodicTabularFunction")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TwoDimensionalTabularFunction<double(double,double)>, MBSIMNS"TwoDimensionalTabularFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TwoDimensionalTabularFunction<VecV(double,double)>, MBSIMNS"TwoDimensionalTabularFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TwoDimensionalTabularFunction<VecV(VecV,VecV)>, MBSIMNS"TwoDimensionalTabularFunction")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TwoDimensionalTabularFunction<Vec(Vec,Vec)>, MBSIMNS"TwoDimensionalTabularFunction")
}
