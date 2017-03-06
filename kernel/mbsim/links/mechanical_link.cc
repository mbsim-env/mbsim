/* Copyright (C) 2004-2016 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/links/mechanical_link.h"
#include "mbsim/frames/frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MechanicalLink::MechanicalLink(const std::string &name) : Link(name), updF(true), updM(true), updRMV(true), updlaF(true), updlaM(true) {
  }

  void MechanicalLink::resetUpToDate() {
    Link::resetUpToDate(); 
    updF = true; 
    updM = true; 
    updRMV = true;
    updlaF = true;
    updlaM = true;
  }

  void MechanicalLink::init(InitStage stage) {
    if(stage==plotting) {
      Link::init(stage);
    }
    else
      Link::init(stage);
  }

  void MechanicalLink::updateGeneralizedForces() {
    lambda.set(iF, evallaF());
    lambda.set(iM, evallaM());
    updla = false;
  }

  void MechanicalLink::plot() {
    Link::plot();
  }

  void MechanicalLink::closePlot() {
    if(getPlotFeature(11334901831169464975ULL)==enabled) {
      Link::closePlot();
    }
  }

  void MechanicalLink::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
  }

}
