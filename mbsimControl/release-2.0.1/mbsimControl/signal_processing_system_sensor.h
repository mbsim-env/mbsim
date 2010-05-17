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

#ifndef _SIGNAL_PROCESSING_SYSTEM_SENSORS_H_
#define _SIGNAL_PROCESSING_SYSTEM_SENSORS_H_

#include "mbsimControl/sensor.h"

namespace MBSimControl {

  class SignalProcessingSystem;

  /*!
   * \brief SignalProcessingSystemSensor
   * \author Markus Schneider
   */
  class SignalProcessingSystemSensor : public Sensor {
    public:
      SignalProcessingSystemSensor(const std::string &name) : Sensor(name), sps(NULL), spsString("") {}
      virtual std::string getType() const {return "SignalProcessingSystemSensor"; }
      void initializeUsingXML(TiXmlElement * element);
      void init(MBSim::InitStage stage);
      
      void setSignalProcessingSystem(SignalProcessingSystem * sps_) {sps=sps_; }
      fmatvec::Vec getSignal();

    private:
      SignalProcessingSystem * sps;
      std::string spsString;
  };

}

#endif /* _SIGNAL_PROCESSING_SYSTEM_SENSORS_H_ */

