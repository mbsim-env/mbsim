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

  /** A dummy Signal class which just feeds a signal throught.
   * The aim of this class is just to mark a signal for external use.
   * Hence e.g. the FMI export or co-simulation searches for all classes of this type
   * and use all these elements as signal outputs of the system. */
  class ExternSignalSink : public Signal {
    protected:
      Signal *signal{nullptr};
      std::string signalString;
    public:
      ExternSignalSink(const std::string &name="") : Signal(name) {}
      void updateSignal() override;
      void setSignal(Signal *sig) { signal=sig; }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      int getSignalSize() const override { return signal->getSignalSize(); }
  };

}

#endif /* _EXTERNSIGNALSINK_H_ */
