/* Copyright (C) 2004-2020 MBSim Development Team
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

#ifndef _MECHANICAL_LINK_SENSORS_H_
#define _MECHANICAL_LINK_SENSORS_H_

#include "mbsimControl/link_sensors.h"
#include "mbsim/utils/index.h"

namespace MBSim {
  class MechanicalLink;
}

namespace MBSimControl {

  /*!
   * \brief MechanicalLinkSensor
   * \author Martin Foerg
   */
  class MechanicalLinkSensor : public LinkSensor {
    public:
      MechanicalLinkSensor(const std::string &name) : LinkSensor(name) { }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
  };

  /*!
   * \brief MechanicalLinkForceSensor
   * \author Martin Foerg
   */
  class MechanicalLinkForceSensor : public MechanicalLinkSensor {
    public:
      MechanicalLinkForceSensor(const std::string &name="") : MechanicalLinkSensor(name) { }
      void setForceNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

  /*!
   * \brief MechanicalLinkMomentSensor
   * \author Martin Foerg
   */
  class MechanicalLinkMomentSensor : public MechanicalLinkSensor {
    public:
      MechanicalLinkMomentSensor(const std::string &name="") : MechanicalLinkSensor(name) { }
      void setMomentNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

}

#endif
