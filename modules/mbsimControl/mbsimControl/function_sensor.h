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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef _FUNCTION_SENSOR_H_
#define _FUNCTION_SENSOR_H_

#include "mbsimControl/sensor.h"
#include <mbsim/functions/function.h>

namespace MBSimControl {

  /*!
   * \brief FunctionSensor
   * \author Markus Schneider
   */
  class FunctionSensor : public Sensor {
    public:
      FunctionSensor(const std::string &name="") : Sensor(name), function(NULL) {}
      FunctionSensor(const std::string &name, MBSim::Function<fmatvec::VecV(double)>* function_);
      ~FunctionSensor() { delete function; }
      std::string getType() const { return "FunctionSensor"; }
      void setFunction(MBSim::Function<fmatvec::VecV(double)>* function_);
      void updateSignal(double t);
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(MBSim::Element::InitStage stage);
      int getSignalSize() const { return (*function)(0).size(); }
    private:
      MBSim::Function<fmatvec::VecV(double)> * function;
  };

}

#endif /* _FUNCTION_SENSOR_H_ */

