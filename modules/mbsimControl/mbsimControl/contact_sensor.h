/* Copyright (C) 2004-2018 MBSim Development Team
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

#ifndef _CONTACT_SENSOR_H_
#define _CONTACT_SENSOR_H_

#include "mbsimControl/sensor.h"
#include "mbsim/utils/index.h"

namespace MBSim {
  class Contact;
}

namespace MBSimControl {

  /*!
   * \brief ContactSensor
   * \author Martin Foerg
   */
  class ContactSensor : public Sensor {
    public:
      ContactSensor(const std::string &name) : Sensor(name) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setContact(MBSim::Contact * contact_) { contact = contact_; }
      void setSingleContactNumber(int i_) { i = i_; }
    protected:
      MBSim::Contact *contact{nullptr};
      std::string contactString;
      MBSim::Index i{0};
  };

  /*!
   * \brief GeneralizedRelativeContactPositionSensor
   * \author Martin Foerg
   */
  class GeneralizedRelativeContactPositionSensor : public ContactSensor {
    public:
      GeneralizedRelativeContactPositionSensor(const std::string &name="") : ContactSensor(name) { }
      int getSignalSize() const override;
      void updateSignal() override;
  };

  /*!
   * \brief GeneralizedRelativeContactVelocitySensor
   * \author Martin Foerg
   */
  class GeneralizedRelativeContactVelocitySensor : public ContactSensor {
    public:
      GeneralizedRelativeContactVelocitySensor(const std::string &name="") : ContactSensor(name) { }
      int getSignalSize() const override;
      void updateSignal() override;
  };

  /*!
   * \brief GeneralizedContactForceSensor
   * \author Martin Foerg
   */
  class GeneralizedContactForceSensor : public ContactSensor {
    public:
      GeneralizedContactForceSensor(const std::string &name="") : ContactSensor(name) { }
      int getSignalSize() const override;
      void updateSignal() override;
  };

}

#endif
