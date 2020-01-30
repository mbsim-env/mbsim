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

#ifndef _CONSTRAINT_SENSORS_H_
#define _CONSTRAINT_SENSORS_H_

#include "mbsimControl/sensor.h"
#include "mbsim/utils/index.h"

namespace MBSim {
  class Constraint;
}

namespace MBSimControl {

  /*!
   * \brief ConstraintSensor
   * \author Martin Foerg
   */
  class ConstraintSensor : public Sensor {
    public:
      ConstraintSensor(const std::string &name) : Sensor(name) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setConstraint(MBSim::Constraint *constraint_) { constraint = constraint_; }
    protected:
      MBSim::Constraint *constraint{nullptr};
      std::string constraintString;
  };

  /*!
   * \brief MechanicalConstraintSensor
   * \author Martin Foerg
   */
  class MechanicalConstraintSensor : public ConstraintSensor {
    public:
      MechanicalConstraintSensor(const std::string &name) : ConstraintSensor(name) { }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
  };

  /*!
   * \brief MechanicalConstraintForceSensor
   * \author Martin Foerg
   */
  class MechanicalConstraintForceSensor : public MechanicalConstraintSensor {
    public:
      MechanicalConstraintForceSensor(const std::string &name="") : MechanicalConstraintSensor(name) { }
      void setForceNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

  /*!
   * \brief MechanicalConstraintMomentSensor
   * \author Martin Foerg
   */
  class MechanicalConstraintMomentSensor : public MechanicalConstraintSensor {
    public:
      MechanicalConstraintMomentSensor(const std::string &name="") : MechanicalConstraintSensor(name) { }
      void setMomentNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

}

#endif
