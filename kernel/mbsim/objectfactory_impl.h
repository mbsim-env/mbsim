/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: friedrich.at.gc@googlemail.com
 */

#ifndef _MBSIM_OBJECTFACTORY_IMPL_H_
#define _MBSIM_OBJECTFACTORY_IMPL_H_

#include "objectfactory.h"

namespace MBSim {

// define the regmap function (include this file only in cc files and that trigger explicit instantation)
template<class EnumType>
std::map<MBXMLUtils::FQN, std::reference_wrapper<const EnumType>>& EnumFactory<EnumType>::regmap() {
  static std::map<MBXMLUtils::FQN, std::reference_wrapper<const EnumType>> reg_;
  return reg_;
}

}

#endif
