/* Copyright (C) 2004-2022 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */


#include <config.h>
#include "mbsimControl/duration.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, Duration)

  void Duration::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    inputSignalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"threshold");
    if(e) setThreshold(E(e)->getText<double>());
  }

  void Duration::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not inputSignalString.empty())
	setInputSignal(getByPath<Signal>(inputSignalString));
      if(not inputSignal)
        throwError("(Duration::init): control input signal is not given!");
    }
    Signal::init(stage, config);
  }

  void Duration::updateSignal() {
    s(0) = active?getTime()-t0:0;
    upds = false;
  }

  void Duration::updateStopVector() {
    sv(0) = inputSignal->evalSignal()(0) - s0;
  }

  void Duration::checkActive(int j) {
    if(j==1) {
      if(inputSignal->evalSignal()(0) >= s0) {
	t0 = getTime();
	active = true;
      }
    }
    else if(j==5) {
      if(jsv(0)) {
	if(not active)
	  t0 = getTime();
	active = not active;
      }
    }
  }

}
