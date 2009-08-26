/* Copyright (C) 2006  Mathias Bachmayer

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
 *
 * Contact:
 *   mbachmayer@gmx.de
 *
 */ 

#ifndef _SIGNAL_PROCESSING_SYSTEM_H_
#define _SIGNAL_PROCESSING_SYSTEM_H_

#include "mbsim/order_one_dynamics.h"

namespace MBSim {

  class Signal;

  /*!
   * \brief SignalProcessingSystem
   * \author Markus Schneider
   */
  class SignalProcessingSystem : public MBSim::OrderOneDynamics {

    public:
      SignalProcessingSystem(const std::string &name) : OrderOneDynamics(name), inputSignal(0) {
      }

      void init(InitStage stage) {}

      void updateg(double t) {}

      virtual fmatvec::Vec calculateOutput() = 0;

      void setInputSignal(Signal * inputSignal_) {inputSignal=inputSignal_; }

      virtual std::string getType() const {return "SignalProcessingSystem"; }

    protected:
      Signal * inputSignal;
  };

}

#endif /* _SIGNAL_PROCESSING_SYSTEM_H_ */

