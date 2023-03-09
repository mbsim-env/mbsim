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

#ifndef _FRAME_SENSORS_H_
#define _FRAME_SENSORS_H_

#include "mbsimControl/sensor.h"

namespace MBSim {
  class Frame;
}

namespace MBSimControl {

  /*!
   * \brief FrameSensor
   * \author Markus Schneider
   */
  class FrameSensor : public Sensor {
    public:
      FrameSensor(const std::string &name) : Sensor(name), frame(nullptr) { }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setFrame(MBSim::Frame * frame_) { frame = frame_; }
      void setOutputFrame(MBSim::Frame *outputFrame_) { outputFrame = outputFrame_; }
      int getSignalSize() const override { return 3; }
    protected:
      MBSim::Frame *frame;
      std::string saved_frame;
      MBSim::Frame *outputFrame { nullptr };
      std::string saved_outputFrame;
  };

  /*!
   * \brief PositionSensor
   * \author Markus Schneider
   */
  class PositionSensor : public FrameSensor {
    public:
      PositionSensor(const std::string &name="") : FrameSensor(name) { }
      void updateSignal() override;
  };

  /*!
   * \brief OrientationSensor
   * \author Martin Foerg
   */
  class OrientationSensor : public FrameSensor {
    public:
      OrientationSensor(const std::string &name="") : FrameSensor(name) { }
      void updateSignal() override;
      int getSignalSize() const override { return 9; }
  };

  /*!
   * \brief VelocitySensor
   * \author Markus Schneider
   */
  class VelocitySensor : public FrameSensor {
    public:
      VelocitySensor(const std::string &name="") : FrameSensor(name) { }
      void updateSignal() override;
  };

  /*!
   * \brief AngularVelocitySensor
   * \author Markus Schneider
   */
  class AngularVelocitySensor : public FrameSensor {
    public:
      AngularVelocitySensor(const std::string &name="") : FrameSensor(name) { }
      void updateSignal() override;
  };

  /*!
   * \brief AccelerationSensor
   * \author Martin Förg
   */
  class AccelerationSensor : public FrameSensor {
    public:
      AccelerationSensor(const std::string &name="") : FrameSensor(name) { }
      void updateSignal() override;
  };

  /*!
   * \brief AngularAccelerationSensor
   * \author Martin Förg
   */
  class AngularAccelerationSensor : public FrameSensor {
    public:
      AngularAccelerationSensor(const std::string &name="") : FrameSensor(name) { }
      void updateSignal() override;
  };

}

#endif /* _FRAME_SENSORS_H_ */
