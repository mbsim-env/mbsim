/* Copyright (C) 2004-2022 MBSim Development Team
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

#ifndef _TYRE_CONTACT_SENSOR_H_
#define _TYRE_CONTACT_SENSOR_H_

#include "mbsimControl/sensor.h"
#include "mbsim/utils/index.h"

namespace MBSim {
  class TyreContact;
}

namespace MBSimControl {

  /*!
   * \brief TyreContactSensor
   * \author Martin Foerg
   */
  class TyreContactSensor : public Sensor {
    public:
      TyreContactSensor(const std::string &name) : Sensor(name) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setTyreContact(MBSim::TyreContact *contact_) { contact = contact_; }
    protected:
      MBSim::TyreContact *contact{nullptr};
      std::string contactString;
  };

  /*!
   * \brief TyreContactPositionSensor
   * \author Martin Foerg
   */
  class TyreContactPositionSensor : public TyreContactSensor {
    public:
      TyreContactPositionSensor(const std::string &name="") : TyreContactSensor(name) { }
      void setPositionNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

  /*!
   * \brief TyreContactOrientationSensor
   * \author Martin Foerg
   */
  class TyreContactOrientationSensor : public TyreContactSensor {
    public:
      TyreContactOrientationSensor(const std::string &name="") : TyreContactSensor(name) { }
      void setOrientationNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 9; }
      void updateSignal() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

  /*!
   * \brief TyreContactVelocitySensor
   * \author Martin Foerg
   */
  class TyreContactVelocitySensor : public TyreContactSensor {
    public:
      TyreContactVelocitySensor(const std::string &name="") : TyreContactSensor(name) { }
      void setVelocityNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

  /*!
   * \brief TyreContactAngularVelocitySensor
   * \author Martin Foerg
   */
  class TyreContactAngularVelocitySensor : public TyreContactSensor {
    public:
      TyreContactAngularVelocitySensor(const std::string &name="") : TyreContactSensor(name) { }
      void setAngularVelocityNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

  /*!
   * \brief TyreModelSensor
   * \author Martin Foerg
   */
  class TyreModelSensor : public TyreContactSensor {
    public:
      TyreModelSensor(const std::string &name="") : TyreContactSensor(name) { }
      int getSignalSize() const override;
      void updateSignal() override;
  };

}

#endif
