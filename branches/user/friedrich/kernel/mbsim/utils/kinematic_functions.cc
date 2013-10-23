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

#include "mbsim/utils/kinematic_functions.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongXAxis<VecV>, MBSIMNS"TranslationAlongXAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongXAxis<double>, MBSIMNS"TranslationAlongXAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongYAxis<VecV>, MBSIMNS"TranslationAlongYAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongYAxis<double>, MBSIMNS"TranslationAlongYAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongZAxis<VecV>, MBSIMNS"TranslationAlongZAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongZAxis<double>, MBSIMNS"TranslationAlongZAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongAxesXY<VecV>, MBSIMNS"TranslationAlongAxesXY")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongAxesYZ<VecV>, MBSIMNS"TranslationAlongAxesYZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongAxesXZ<VecV>, MBSIMNS"TranslationAlongAxesXZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongAxesXYZ<VecV>, MBSIMNS"TranslationAlongAxesXYZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongFixedAxis<VecV>, MBSIMNS"TranslationAlongFixedAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, TranslationAlongFixedAxis<double>, MBSIMNS"TranslationAlongFixedAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearTranslation<VecV>, MBSIMNS"LinearTranslation")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, LinearTranslation<double>, MBSIMNS"LinearTranslation")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutXAxis<VecV>, MBSIMNS"RotationAboutXAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutXAxis<double>, MBSIMNS"RotationAboutXAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutYAxis<VecV>, MBSIMNS"RotationAboutYAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutYAxis<double>, MBSIMNS"RotationAboutYAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutZAxis<VecV>, MBSIMNS"RotationAboutZAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutZAxis<double>, MBSIMNS"RotationAboutZAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutFixedAxis<VecV>, MBSIMNS"RotationAboutFixedAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutFixedAxis<double>, MBSIMNS"RotationAboutFixedAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutAxesXY<VecV>, MBSIMNS"RotationAboutAxesXY")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutAxesYZ<VecV>, MBSIMNS"RotationAboutAxesYZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutAxesXZ<VecV>, MBSIMNS"RotationAboutAxesXZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FunctionBase, RotationAboutAxesXYZ<VecV>, MBSIMNS"RotationAboutAxesXYZ")

}
