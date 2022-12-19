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
#include "mbsimControl/state_machine_sensor.h"
#include "mbsimControl/state_machine.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, StateMachineSensor)

  void StateMachineSensor::initializeUsingXML(DOMElement *element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"stateMachine");
    stateMachineString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"state");
    if(e) {
      auto state = E(e)->getText<string>();
      setState(state.substr(1,state.size()-2));
    }
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"selection");
    if(e) {
      string selectionStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(selectionStr=="value") selection=value;
      else if(selectionStr=="activity") selection=activity;
      else if(selectionStr=="durationOfActivity") selection=durationOfActivity;
      else selection=unknownSelection;
    }
  }

  void StateMachineSensor::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not stateMachineString.empty())
	setStateMachine(getByPath<StateMachine>(stateMachineString));
      if(not stateMachine)
        throwError("(StateMachineSensor::init): state machine is not given!");
      if(selection==unknownSelection)
	throwError("(StateMachineSensor::init): selection unknown");
    }
    Sensor::init(stage, config);
  }

  void StateMachineSensor::updateSignal() {
    if(selection==durationOfActivity) {
      if(state.empty() or stateMachine->getActiveState().name==state)
	s(0) = getTime()-stateMachine->getActiveState().t0;
      else
	s(0) = 0;
    }
    if(selection==activity) {
      if(state.empty() or stateMachine->getActiveState().name==state)
	s(0) = 1;
      else
	s(0) = 0;
    }
    else if(selection==value) {
      if(state.empty() or stateMachine->getActiveState().name==state)
	s(0) = stateMachine->getActiveState().val;
      else
	s(0) = 0;
    }
    upds = false;
  }

}
