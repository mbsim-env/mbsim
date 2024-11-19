/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2017 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _NAMESPACE_H_
#define _NAMESPACE_H_

#include <xercesc/util/XercesDefs.hpp>
#include <mbxmlutilshelper/dom.h>

namespace MBSimGUI {

  const MBXMLUtils::NamespaceURI MBSIM("http://www.mbsim-env.de/MBSim", {"", "m", "mbs", "mbsim"});
  const MBXMLUtils::NamespaceURI OPENMBV("http://www.mbsim-env.de/OpenMBV", {"o", "ombv"});
  const MBXMLUtils::NamespaceURI MBSIMCONTROL("http://www.mbsim-env.de/MBSimControl", {"mc", "mbsimcontrol"});
  const MBXMLUtils::NamespaceURI MBSIMFLEX("http://www.mbsim-env.de/MBSimFlexibleBody", {"mf", "mbsimflex"});
  const MBXMLUtils::NamespaceURI MBSIMPHYSICS("http://www.mbsim-env.de/MBSimPhysics", {"mp", "mbsimphy"});
  const MBXMLUtils::NamespaceURI MBSIMFCL("http://www.mbsim-env.de/MBSimFcl", {"mfcl", "mbsimfcl"});
  const MBXMLUtils::NamespaceURI MBSIMXML("http://www.mbsim-env.de/MBSimXML", {"mx", "mbsx", "mbsimxml"});

}

#endif
