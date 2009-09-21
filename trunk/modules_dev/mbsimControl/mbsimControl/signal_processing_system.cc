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
 * Contact: schneidm@users.berlios.de
 */

#include "mbsimControl/signal_processing_system.h"
#include "mbsimControl/signal_.h"
#include "mbsimControl/objectfactory.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  SignalProcessingSystem::SignalProcessingSystem(const string &name) : ExtraDynamic(name), inputSignal(NULL), inputSignalString("") {
  }

  Signal * SignalProcessingSystem::getSignalByPath(string path) {
    int pos=path.find("Signal");
    path.erase(pos, 6);
    path.insert(pos, "Link");
    Link * h = getLinkByPath(path);
    if (dynamic_cast<Signal *>(h))
      return static_cast<Signal *>(h);
    else {
      std::cerr << "ERROR! \"" << path << "\" is not of Signal-Type." << std::endl; 
      _exit(1);
    }
  }

  void SignalProcessingSystem::initializeUsingXML(TiXmlElement * element) {
    ExtraDynamic::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMCONTROLNS"inputSignal");
    inputSignalString=e->Attribute("ref");
  }

  void SignalProcessingSystem::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (inputSignalString!="") {
        Signal * s = getSignalByPath(inputSignalString);
        setInputSignal(s);
      }
      ExtraDynamic::init(stage);
    }
    else
      ExtraDynamic::init(stage);
  }


}
