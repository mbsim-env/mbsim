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
#include "distributing_ffr_interface_node_frame.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements_ffr_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, DistributingFfrInterfaceNodeFrame)

  void DistributingFfrInterfaceNodeFrame::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      map<int,double> w = static_cast<FiniteElementsFfrBody*>(parent)->getWeightingFactors(elements,faceNumber);
      nodes.resize(w.size(),NONINIT);
      weights.resize(w.size(),NONINIT);
      int j=0;
      for(auto & i : w) {
        nodes(j) = i.first;
	weights(j++) = i.second;
      }
    }
    GenericFfrInterfaceNodeFrame::init(stage,config);
  }

  void DistributingFfrInterfaceNodeFrame::initializeUsingXML(DOMElement *element) {
    GenericFfrInterfaceNodeFrame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"elementNumbers");
    setElementNumbers(E(e)->getText<VecVI>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"faceNumber");
    setFaceNumber(E(e)->getText<int>());
  }

}
