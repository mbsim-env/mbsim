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
      LinkSensor(const std::string &name) : Sensor(name), link(nullptr) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setLink(MBSim::Link * link_) { link=link_; }
    protected:
      MBSim::Link * link;
      std::string linkString;
  };

  /*!
   * \brief GeneralizedRelativePositionSensor
   * \author Markus Schneider
   */
  class GeneralizedRelativePositionSensor : public LinkSensor {
    public:
      GeneralizedRelativePositionSensor(const std::string &name="") : LinkSensor(name) {}
      int getSignalSize() const override;
      void updateSignal() override;
  };

  /*!
   * \brief AbsoluteVelocitySensor
   * \author Markus Schneider
   */
  class GeneralizedRelativeVelocitySensor : public LinkSensor {
    public:
      GeneralizedRelativeVelocitySensor(const std::string &name="") : LinkSensor(name) {}
      int getSignalSize() const override;
      void updateSignal() override;
  };

}

#endif /* _LINK_SENSORS_H_ */
