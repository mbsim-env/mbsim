/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef _LINK_SENSORS_H_
#define _LINK_SENSORS_H_

#include "mbsimControl/sensor.h"

namespace MBSim {
  class Link;
}

namespace MBSimControl {

  /*!
   * \brief LinkSensor
   * \author Markus Schneider
   */
  class LinkSensor : public Sensor {
    public:
      LinkSensor(const std::string &name) : Sensor(name), link(NULL), linkString("") {}
      std::string getType() const { return "LinkSensor"; }
      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setLink(MBSim::Link * link_) {link=link_; }
    protected:
      MBSim::Link * link;
      std::string linkString;
  };

  /*!
   * \brief LinkDistanceSensor
   * \author Markus Schneider
   */
  class LinkDistanceSensor : public LinkSensor {
    public:
      LinkDistanceSensor(const std::string &name="") : LinkSensor(name) {}
      std::string getType() const { return "LinkDistanceSensor"; }
      fmatvec::Vec getSignal();
  };

  /*!
   * \brief AbsoluteVelocitySensor
   * \author Markus Schneider
   */
  class LinkVelocitySensor : public LinkSensor {
    public:
      LinkVelocitySensor(const std::string &name="") : LinkSensor(name) {}
      std::string getType() const { return "LinkVelocitySensor"; }
      fmatvec::Vec getSignal();
  };

}

#endif /* _LINK_SENSORS_H_ */
