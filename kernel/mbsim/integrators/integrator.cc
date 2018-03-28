/* Copyright (C) 2004-2014 MBSim Development Team
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
#include <cstdlib>
#include "integrator.h"
#include "mbsim/element.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimIntegrator {

  Integrator::Integrator() :  name("Integrator") {}

  void Integrator::initializeUsingXML(DOMElement *element) {
    Solver::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"startTime");
    setStartTime(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"endTime");
    setEndTime(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotStepSize");
    setPlotStepSize(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"initialState");
    if(e) setInitialState(E(e)->getText<fmatvec::Vec>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"plotIntegrationData");
    if(e) setPlotIntegrationData(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMINT%"writeIntegrationSummary");
    if(e) setWriteIntegrationSummary(E(e)->getText<bool>());
  }

  // This function is called first by each implementation of Integrator::integrate.
  // We modify here some integrator date for debugging (valgrind) purposes.
  void Integrator::debugInit() {
    // set a minimal end time: integrate only up to the first plot time (+10%) after the plot at tStart
    if(getenv("MBSIM_SET_MINIMAL_TEND")!=nullptr)
      setEndTime(getStartTime()+1.1*getPlotStepSize());
  }

}
