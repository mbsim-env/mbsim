/* Copyright (C) 2004-2020 MBSim Development Team
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
#include <iostream>
#include "mbsimControl/nonlinear_transfer_system.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, NonlinearTransferSystem)

  void NonlinearTransferSystem::initializeUsingXML(DOMElement * element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    inputSignalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"systemFunction");
    setSystemFunction(ObjectFactory::createAndInit<MBSim::Function<VecV(VecV,VecV)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"outputFunction");
    setOutputFunction(ObjectFactory::createAndInit<MBSim::Function<VecV(VecV,VecV)>>(e->getFirstElementChild()));
  }

  void NonlinearTransferSystem::updateSignal() {
    s = (*H)(x,inputSignal->evalSignal());
    upds = false;
  }

  void NonlinearTransferSystem::updatexd() {
    xd = (*F)(x,inputSignal->evalSignal());
  }

  void NonlinearTransferSystem::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not inputSignalString.empty())
        setInputSignal(getByPath<Signal>(inputSignalString));
      if(not inputSignal)
        throwError("(NonlinearTransferSystem::init): input signal must be given");
    }
    else if(stage==preInit) {
      if(not F->getRetSize().first)
        throwError("Size of system function must be at least 1");
      if(F->getArg1Size() != F->getRetSize().first)
        throwError("Size of first argument of system function must be equal to size of system function");
      if(F->getArg2Size() != inputSignal->getSignalSize())
        throwError("Size of second argument of system function must be equal to input signal size");
      if(H->getArg1Size() != F->getRetSize().first)
        throwError("Size of first argument of output function must be equal to size of system function");
      if(H->getArg2Size() != inputSignal->getSignalSize())
        throwError("Size of second argument of output function must be equal to input signal size");
    }
    Signal::init(stage, config);
    if(F) F->init(stage, config);
    if(H) H->init(stage, config);
  }

}
