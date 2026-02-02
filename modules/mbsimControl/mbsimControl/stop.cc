/* Copyright (C) 2004-2024 MBSim Development Team
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
#include "mbsimControl/stop.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, Stop)

  void Stop::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    signalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"threshold");
    if(e) setThreshold(E(e)->getText<VecV>());
  }

  void Stop::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not signalString.empty())
        setInputSignal(getByPath<Signal>(signalString));
      if(not signal)
        throwError("(Stop::init): input signal is not given!");
    }
    else if(stage==preInit) {
      if(s0.size()==0)
	s0.resize(signal->getSignalSize());
      if(signal->getSignalSize() != s0.size())
        throwError("(Stop::init): size of threshold must be equal to size of input signal!");
    }
    Signal::init(stage, config);
  }

  void Stop::updateSignal() {
    s = signal->evalSignal();
    upds = false;
  }

  void Stop::updateStopVector() {
    sv = signal->evalSignal() - s0;
  }

  void Stop::checkActive(int j) {
    if(j==5) {
      for(int i=0; i<jsv.size(); i++) {
        if(jsv(i))
          throwError("(Stop::checkActive): stop!");;
      }
    }
  }

}
