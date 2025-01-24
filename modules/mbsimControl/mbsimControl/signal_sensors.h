/* Copyright (C) 2004-2025 MBSim Development Team
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

#ifndef _SIGNAL_SENSORS_H_
#define _SIGNAL_SENSORS_H_

#include "mbsimControl/sensor.h"

namespace MBSimControl {

  class Signal;

  /*!
   * \brief SignalSensor
   * \author Martin Förg
   */
  class SignalSensor : public Sensor {
    public:
      SignalSensor(const std::string &name) : Sensor(name), signal(nullptr) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setSignal(Signal *signal_) { signal = signal_; }
      int getSignalSize() const override; 
    protected:
      Signal *signal;
      std::string saved_signal;
  };

}

#endif
