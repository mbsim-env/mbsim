/* Copyright (C) 2004-2019 MBSim Development Team
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
#include "ffr_interface_node_frame.h"
#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"

using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FfrInterfaceNodeFrame)

  void FfrInterfaceNodeFrame::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(weights.size()==0)
        weights.resize(nodes.size(),INIT,1.0);
    }
    GenericFfrInterfaceNodeFrame::init(stage,config);
  }

  void FfrInterfaceNodeFrame::initializeUsingXML(DOMElement *element) {
    GenericFfrInterfaceNodeFrame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumbers");
    setNodeNumbers(E(e)->getText<VecVI>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"weightingFactors");
    if(e) setWeightingFactors(E(e)->getText<VecV>());
  }

}
