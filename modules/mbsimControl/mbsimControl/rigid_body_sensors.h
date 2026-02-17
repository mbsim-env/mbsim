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

#ifndef _RIGID_BODY_SENSORS_H_
#define _RIGID_BODY_SENSORS_H_

#include "mbsimControl/object_sensors.h"
#include "mbsim/utils/index.h"

namespace MBSimControl {

  /*!
   * \brief RigidBodySensor
   * \author Martin Foerg
   */
  class RigidBodySensor : public ObjectSensor {
    public:
      RigidBodySensor(const std::string &name) : ObjectSensor(name) { }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
  };

  /*!
   * \brief RigidBodyJointForceSensor
   * \author Martin Foerg
   */
  class RigidBodyJointForceSensor : public RigidBodySensor {
    public:
      RigidBodyJointForceSensor(const std::string &name="") : RigidBodySensor(name) { }
      void setJointForceNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

  /*!
   * \brief RigidBodyJointMomentSensor
   * \author Martin Foerg
   */
  class RigidBodyJointMomentSensor : public RigidBodySensor {
    public:
      RigidBodyJointMomentSensor(const std::string &name="") : RigidBodySensor(name) { }
      void setJointMomentNumber(int i_) { i = i_; }
      int getSignalSize() const override { return 3; }
      void updateSignal() override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
    protected:
      MBSim::Index i{0};
  };

}

#endif
