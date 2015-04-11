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
 * Contact: friedrich.at.gc@googlemail.com
 */

#include "config.h"
#include "mbsimControl/extern_signal_sink.h"

using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ExternSignalSink, MBSIMCONTROL%"ExternSignalSink")

  void ExternSignalSink::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
  }

  void ExternSignalSink::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      setSignal(getByPath<Signal>(signalString));
      Signal::init(stage);
    }
    else if(stage==preInit) {
      Link::init(stage);
      addDependency(signal);
    }
    else
      Link::init(stage);
  }

  void ExternSignalSink::updateh(double t, int j) {
    s = signal->getSignal();
  }

}
