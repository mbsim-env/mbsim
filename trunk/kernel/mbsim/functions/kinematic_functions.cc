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

#include "mbsim/functions/kinematic_functions.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongXAxis<VecV>, MBSIM%"TranslationAlongXAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongXAxis<double>, MBSIM%"TranslationAlongXAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongYAxis<VecV>, MBSIM%"TranslationAlongYAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongYAxis<double>, MBSIM%"TranslationAlongYAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongZAxis<VecV>, MBSIM%"TranslationAlongZAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongZAxis<double>, MBSIM%"TranslationAlongZAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongAxesXY<VecV>, MBSIM%"TranslationAlongAxesXY")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongAxesYZ<VecV>, MBSIM%"TranslationAlongAxesYZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongAxesXZ<VecV>, MBSIM%"TranslationAlongAxesXZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongAxesXYZ<VecV>, MBSIM%"TranslationAlongAxesXYZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongFixedAxis<VecV>, MBSIM%"TranslationAlongFixedAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(TranslationAlongFixedAxis<double>, MBSIM%"TranslationAlongFixedAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(LinearTranslation<VecV>, MBSIM%"LinearTranslation")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(LinearTranslation<double>, MBSIM%"LinearTranslation")

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutXAxis<VecV>, MBSIM%"RotationAboutXAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutXAxis<double>, MBSIM%"RotationAboutXAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutYAxis<VecV>, MBSIM%"RotationAboutYAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutYAxis<double>, MBSIM%"RotationAboutYAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutZAxis<VecV>, MBSIM%"RotationAboutZAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutZAxis<double>, MBSIM%"RotationAboutZAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutFixedAxis<VecV>, MBSIM%"RotationAboutFixedAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutFixedAxis<double>, MBSIM%"RotationAboutFixedAxis")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutAxesXY<VecV>, MBSIM%"RotationAboutAxesXY")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutAxesYZ<VecV>, MBSIM%"RotationAboutAxesYZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutAxesXZ<VecV>, MBSIM%"RotationAboutAxesXZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutAxesXYZ<VecV>, MBSIM%"RotationAboutAxesXYZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutAxesZXZ<VecV>, MBSIM%"RotationAboutAxesZXZ")
  MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(RotationAboutAxesZYX<VecV>, MBSIM%"RotationAboutAxesZYX")

}
