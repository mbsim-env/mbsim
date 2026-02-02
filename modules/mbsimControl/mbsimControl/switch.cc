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
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"firstDataInputSignal");
    dataSignalString1=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"secondDataInputSignal");
    dataSignalString2=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"controlInputSignal");
    controlSignalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"threshold");
    if(e) setThreshold(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"rootFinding");
    if(e) setRootFinding(E(e)->getText<bool>());
  }

  void Switch::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not dataSignalString1.empty())
        setFirstDataInputSignal(getByPath<Signal>(dataSignalString1));
      if(not dataSignalString2.empty())
        setSecondDataInputSignal(getByPath<Signal>(dataSignalString2));
      if(not controlSignalString.empty())
        setControlInputSignal(getByPath<Signal>(controlSignalString));
      if(not dataSignal1)
        throwError("(Switch::init): first data input signal is not given!");
      if(not dataSignal2)
        throwError("(Switch::init): second data input signal is not given!");
      if(not controlSignal)
        throwError("(Switch::init): control input signal is not given!");
    }
    else if(stage==preInit) {
      if(dataSignal1->getSignalSize() != dataSignal2->getSignalSize())
        throwError("(Switch::init): size of first data input signal must be equal to size of second data input signal.");
      if(controlSignal->getSignalSize() != 1)
        throwError("(Switch::init): size of control input signal must be equal to 1.");
    }
    Signal::init(stage, config);
  }

  void Switch::updateSignal() {
    s = (rf?active:controlSignal->evalSignal()(0)>=s0)?dataSignal1->evalSignal():dataSignal2->evalSignal();
    upds = false;
  }

  void Switch::updateStopVector() {
    sv(0) = controlSignal->evalSignal()(0) - s0;
  }

  void Switch::checkActive(int j) {
    if(j==1)
      active = controlSignal->evalSignal()(0) >= s0;
    else if(j==5) {
      if(jsv(0))
	active = not active;
    }
  }

}
