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
#include "mbxmlutilshelper/dom.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;

namespace MBSim {

pair<reference_wrapper<const PlotFeatureEnum>, PlotFeatureStatus> getPlotFeatureFromXML(const xercesc::DOMElement* e) {
  // get string
  string str(E(e)->getAttribute("feature"));
  if(str.empty())
    throw DOMEvalException("Empty string not allowed for PlotFeature", e);
  // get status
  PlotFeatureStatus enumStatus;
  switch(str[0]) {
    case '+': enumStatus=enabled;  break;
    case '-': enumStatus=disabled; break;
    default: throw DOMEvalException("PlotFeature must start with '+' or '-'.", e);
  }
  // get num
  const PlotFeatureEnum& enumValue=EnumFactory<PlotFeatureEnum>::get(str.substr(1), e);
  // return
  return make_pair(ref(enumValue), enumStatus);
}

}
