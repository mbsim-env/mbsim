/* Copyright (C) 2004-2021 MBSim Development Team
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
#include "mbsimControl/switch.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, Switch)

  void Switch::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"firstInputSignal");
    inputSignalString1=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"secondInputSignal");
    inputSignalString2=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"controlSignal");
    controlSignalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"threshold");
    if(e) setThreshold(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"rootFinding");
    if(e) setRootFinding(E(e)->getText<bool>());
  }

  void Switch::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not inputSignalString1.empty())
	setFirstInputSignal(getByPath<Signal>(inputSignalString1));
      if(not inputSignalString2.empty())
	setSecondInputSignal(getByPath<Signal>(inputSignalString2));
      if(not controlSignalString.empty())
	setControlSignal(getByPath<Signal>(controlSignalString));
      if(not inputSignal1)
        throwError("First input input signal is not given!");
      if(not inputSignal2)
        throwError("Second input input signal is not given!");
      if(not controlSignal)
        throwError("Control input signal is not given!");
    }
    else if(stage==preInit) {
      if(inputSignal1->getSignalSize() != inputSignal2->getSignalSize())
        throwError("Size of first input signal must be equal to size of second input signal");
      if(controlSignal->getSignalSize() != 1)
        throwError("Size of control input signal must be equal to 1");
    }
    Signal::init(stage, config);
  }

  void Switch::updateSignal() {
    s = controlSignal->evalSignal()(0)>=s0?inputSignal1->evalSignal():inputSignal2->evalSignal();
    upds = false;
  }

  void Switch::updateStopVector() {
    sv(0) = controlSignal->evalSignal()(0) - s0;
  }

}
