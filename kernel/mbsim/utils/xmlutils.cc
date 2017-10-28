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
#include "mbsim/utils/xmlutils.h"
#include "mbsim/element.h"
#include "mbxmlutilshelper/dom.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;

namespace MBSim {

pair<reference_wrapper<const PlotFeatureEnum>, bool> getPlotFeatureFromXML(const xercesc::DOMElement* e) {
  FQN fqn(A(E(e)->getAttributeNode("value"))->getQName());
  const PlotFeatureEnum& enumValue=EnumFactory<PlotFeatureEnum>::get(fqn, e);

  bool enumStatus=Element::getBool(e);

  return make_pair(ref(enumValue), enumStatus);
}

}
