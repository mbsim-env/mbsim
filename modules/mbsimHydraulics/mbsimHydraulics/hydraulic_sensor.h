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

#ifndef  _HYDRAULIC_SENSOR_H_
#define  _HYDRAULIC_SENSOR_H_

#include "mbsimControl/sensor.h"

namespace MBSimHydraulics {

  class HNode;
  class HLine;

  class FlowSensor : public MBSimControl::Sensor {
    public:
      FlowSensor(const std::string &name) : MBSimControl::Sensor(name), line(NULL), lineString("") {}
      std::string getType() const { return "FlowSensor"; }
      fmatvec::Vec getSignal();
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setHLine(HLine * line_) {line=line_; }
    protected:
      HLine * line;
      std::string lineString;
  };

  class PressureSensor : public MBSimControl::Sensor {
    public:
      PressureSensor(const std::string &name) : MBSimControl::Sensor(name), node(NULL), nodeString("") {}
      std::string getType() const { return "PressureSensor"; }
      fmatvec::Vec getSignal();
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setHNode(HNode * node_) {node=node_; }
    protected:
      HNode * node;
      std::string nodeString;
  };

  class TemperatureSensor : public MBSimControl::Sensor {
    public:
      TemperatureSensor(const std::string &name) : MBSimControl::Sensor(name), T(1) {}
      std::string getType() const { return "TemperatureSensor"; }
      fmatvec::Vec getSignal() {return T; }
      void init(MBSim::InitStage stage);
    private:
      fmatvec::Vec T;
  };

  class KinematicViscositySensor : public MBSimControl::Sensor {
    public:
      KinematicViscositySensor(const std::string &name) : MBSimControl::Sensor(name), nu(1) {}
      std::string getType() const { return "KinematicViscositySensor"; }
      fmatvec::Vec getSignal() {return nu; }
      void init(MBSim::InitStage stage);
    private:
      fmatvec::Vec nu;
  };

}

#endif   /* ----- #ifndef _HYDRAULIC_SENSOR_H_  ----- */

