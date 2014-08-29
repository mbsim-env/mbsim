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

#ifndef _EXTERNSIGNALSINK_H_
#define _EXTERNSIGNALSINK_H_

#include "mbsimControl/signal_.h"

namespace MBSimControl {

  /** Signal which value is given by an external resource. E.g. Cosimulation **/
  class ExternSignalSink : public Signal {
    protected:
      fmatvec::Vec sink;
      int sinkSize;
    public:
      ExternSignalSink(const std::string &name="") : Signal(name), sinkSize(0) {}
      void setSinkSize(int size) { sinkSize=size; sink.resize(sinkSize); }
      std::string getType() const { return "ExternSignalSink"; }
      fmatvec::Vec getSignal() { return sink; }
      void initializeUsingXML(xercesc::DOMElement *element) {
        Signal::initializeUsingXML(element);
        setSinkSize(getInt(MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMCONTROL%"sinkSize")));
      }
  };

}

#endif /* _EXTERNSIGNALSINK_H_ */
