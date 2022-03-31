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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef _SWITCH_H_
#define _SWITCH_H_

#include "mbsimControl/signal_.h"

namespace MBSimControl {

  /*!
   * \brief Switch
   * \author Martin Foerg
   */
  class Switch : public Signal {
    public:
      Switch(const std::string &name="") : Signal(name) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setFirstInputSignal(Signal *signal_) { inputSignal1 = signal_; }
      void setSecondInputSignal(Signal *signal_) { inputSignal2 = signal_; }
      void setControlSignal(Signal *signal_) { controlSignal = signal_; }
      void setThreshold(double s0_) { s0 = s0_; }
      void setRootFinding(bool rf_) { rf = rf_; }
      void updateSignal() override;
      int getSignalSize() const override { return inputSignal1->getSignalSize(); }
      bool isSetValued() const override { return rf; }
      void calcsvSize() override { svSize = isSetValued(); }
      void updateStopVector() override;
    private:
      Signal* inputSignal1{nullptr};
      Signal* inputSignal2{nullptr};
      Signal* controlSignal{nullptr};
      double s0{0};
      bool rf{true};
      std::string inputSignalString1, inputSignalString2, controlSignalString;
  };

}

#endif
