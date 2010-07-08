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
 * Contact: mafriedrich@users.berlios.de
 */

#ifndef _EXTERNSIGNALSOURCE_H_
#define _EXTERNSIGNALSOURCE_H_

#include "mbsimControl/signal_.h"
#include "mbsimControl/objectfactory.h"

namespace MBSimControl {

  /** Signal which value if given by a external resource. E.g. Cosimulation */
  class ExternSignalSource : public Signal {
    protected:
      fmatvec::Vec source;
      int sourceSize;
    public:
      ExternSignalSource(const std::string &name) : Signal(name), sourceSize(0) {}
      void setSourceSize(int size) { sourceSize=size; source.resize(sourceSize); }
      std::string getType() const { return "ExternSignalSource"; }
      fmatvec::Vec getSignal() { return source; }
      void setSignal(fmatvec::Vec s) { assert(s.size()==source.size()); source=s; }
      void initializeUsingXML(TiXmlElement *element) {
        Signal::initializeUsingXML(element);
        setSourceSize(getInt(element->FirstChildElement(MBSIMCONTROLNS"sourceSize")));
      }
  };

}

#endif /* _SENSOR_H_ */
