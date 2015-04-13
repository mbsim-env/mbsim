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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimControl/signal_processing_system.h"
#include "mbsimControl/signal_.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  SignalProcessingSystem::SignalProcessingSystem(const string &name) : Link(name), inputSignal(NULL), inputSignalString("") {
    setPlotFeature(globalPosition, enabled);
    setPlotFeature(state, enabled);
  }

  void SignalProcessingSystem::initializeUsingXML(DOMElement * element) {
    Link::initializeUsingXML(element);
    DOMElement * e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    inputSignalString=E(e)->getAttribute("ref");
  }

  void SignalProcessingSystem::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (inputSignalString!="")
        setInputSignal(getByPath<Signal>(inputSignalString));
      Link::init(stage);
    }
    else if(stage==preInit) {
      Link::init(stage);
      addDependency(inputSignal);
    }
    else
      Link::init(stage);
  }

     int SignalProcessingSystem::getSignalSize() const {return inputSignal->getSignalSize(); }

}
