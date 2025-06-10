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
#include "mbsimControl/extern_signal_source.h"

using namespace MBXMLUtils;
using namespace xercesc;
using namespace fmatvec;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, ExternSignalSource)

  void ExternSignalSource::init(InitStage stage, const MBSim::InitConfigSet &config) {
    Signal::init(stage, config);
    if(stage==unknownStage)
      // initialize external signal with 0 (required to avoid an undefined value if the signal is not set)
      s=VecV(getSignalSize(), INIT, 0.0);
  }

  void ExternSignalSource::initializeUsingXML(DOMElement *element) {
    Signal::initializeUsingXML(element);
    setSourceSize(E(E(element)->getFirstElementChildNamed(MBSIMCONTROL%"sourceSize"))->getText<int>());
  }

}

namespace MBSim {
  template<> MBSimControl::ExternSignalSource* Element::getByPath(const std::string &path, bool initialCaller) const {
    Element *e = getByPathElement(path, initialCaller);
    auto *t=dynamic_cast<MBSimControl::ExternSignalSource*>(e);
    if(t)
      return t;
    else
      throwError(std::string("Cannot cast this element to type ")+typeid(MBSimControl::ExternSignalSource).name()+".");
  }
}

