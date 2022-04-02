/* Copyright (C) 2004-2021 MBSim Development Team
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

#ifndef _DURATION_H_
#define _DURATION_H_

#include "mbsimControl/signal_.h"

namespace MBSimControl {

  /*!
   * \brief Duration
   * \author Martin Foerg
   */
  class Duration : public Signal {
    public:
      Duration(const std::string &name="") : Signal(name) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setInputSignal(Signal *signal_) { inputSignal = signal_; }
      void setThreshold(double s0_) { s0 = s0_; }
      void updateSignal() override;
      int getSignalSize() const override { return 1; }
      bool isSetValued() const override { return true; }
      void calcsvSize() override { svSize = isSetValued(); }
      void updateStopVector() override;
      void checkActive(int j) override;
      bool isActive() const override { return active; }
    private:
      Signal* inputSignal{nullptr};
      double s0{0};
      double t0{0};
      bool active{false};
      std::string inputSignalString;
  };

}

#endif
