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
      FrameSensor(const std::string &name) : Sensor(name), frame(NULL) { }
      std::string getType() const { return "FrameSensor"; }
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(InitStage stage);
      void setFrame(MBSim::Frame * frame_) { frame = frame_; }
      int getSignalSize() const { return 3; }
    protected:
      MBSim::Frame *frame;
      std::string frameString;
  };

  /*!
   * \brief PositionSensor
   * \author Markus Schneider
   */
  class PositionSensor : public FrameSensor {
    public:
      PositionSensor(const std::string &name="") : FrameSensor(name) { }
      std::string getType() const { return "PositionSensor"; }
      void updateSignal();
  };

  /*!
   * \brief OrientationSensor
   * \author Martin Foerg
   */
  class OrientationSensor : public FrameSensor {
    public:
      OrientationSensor(const std::string &name="") : FrameSensor(name) { }
      std::string getType() const { return "OrientationSensor"; }
      void updateSignal();
      int getSignalSize() const { return 9; }
  };

  /*!
   * \brief VelocitySensor
   * \author Markus Schneider
   */
  class VelocitySensor : public FrameSensor {
    public:
      VelocitySensor(const std::string &name="") : FrameSensor(name) { }
      std::string getType() const { return "VelocitySensor"; }
      void updateSignal();
  };

  /*!
   * \brief AngularVelocitySensor
   * \author Markus Schneider
   */
  class AngularVelocitySensor : public FrameSensor {
    public:
      AngularVelocitySensor(const std::string &name="") : FrameSensor(name) { }
      std::string getType() const { return "AngularVelocitySensor"; }
      void updateSignal();
  };
  
//  /*!
//   * \brief RelativeCoordinateSensor
//   * \author Markus Schneider
//   */
//  class RelativeCoordinateSensor : public Sensor {
//    public:
//      RelativeCoordinateSensor(const std::string &name) : Sensor(name), refFrame(NULL), relFrame(NULL), direction(), refFrameString(""), relFrameString("") {}
//      std::string getType() const { return "RelativeCoordinateSensor"; }
//      void initializeUsingXML(xercesc::DOMElement *element);
//      void init(InitStage stage);
//      void setReferenceFrame(MBSim::Frame * refFrame_) {refFrame=refFrame_; }
//      void setRelativeFrame(MBSim::Frame * relFrame_) {relFrame=relFrame_; }
//      void setDirection(fmatvec::Mat direction_) {
//        direction=direction_;
//        for (int i=0; i<direction_.cols(); i++)
//          direction.col(i)=direction.col(i)/nrm2(direction.col(i));
//        assert(direction.rows()==3);
//      }
//      int getSignalSize() const { return direction.cols(); }
//    protected:
//      MBSim::Frame * refFrame;
//      MBSim::Frame * relFrame;
//      fmatvec::Mat direction;
//      std::string refFrameString, relFrameString;
//  };
//
//  /*!
//   * \brief RelativePositionSensor
//   * \author Markus Schneider
//   */
//  class RelativePositionSensor : public RelativeCoordinateSensor {
//    public:
//      RelativePositionSensor(const std::string &name="") : RelativeCoordinateSensor(name) {}
//      std::string getType() const { return "RelativePositionSensor"; }
//      void updateSignal();
//  };
//
//  /*!
//   * \brief RelativeVelocitySensor
//   * \author Markus Schneider
//   */
//  class RelativeVelocitySensor : public RelativeCoordinateSensor {
//    public:
//      RelativeVelocitySensor(const std::string &name="") : RelativeCoordinateSensor(name) {}
//      std::string getType() const { return "RelativeVelocitySensor"; }
//      void updateSignal();
//  };
//
//  /*!
//   * \brief RelativeAngularPositionSensor
//   * \author Markus Schneider
//   */
//  class RelativeAngularPositionSensor : public RelativeCoordinateSensor {
//    public:
//      RelativeAngularPositionSensor(const std::string &name="") : RelativeCoordinateSensor(name) {}
//      std::string getType() const { return "RelativeAngularPositionSensor"; }
//      void updateSignal();
//
//      void calcxSize() {xSize=direction.cols(); }
//      void init(InitStage stage) {
//        if (stage==preInit) {
//          RelativeCoordinateSensor::init(stage);
//          g.resize(direction.cols());
//          gd.resize(direction.cols());
//          x.resize(direction.cols());
//        }
//        else
//          RelativeCoordinateSensor::init(stage);
//      }
//      void updatexd();
//      void updatedx();
//  };
//
//  /*!
//   * \brief RelativeAngularVelocitySensor
//   * \author Markus Schneider
//   */
//  class RelativeAngularVelocitySensor : public RelativeCoordinateSensor {
//    public:
//      RelativeAngularVelocitySensor(const std::string &name="") : RelativeCoordinateSensor(name) {}
//      std::string getType() const { return "RelativeAngularVelocitySensor"; }
//      void updateSignal();
//  };

}

#endif /* _FRAME_SENSORS_H_ */
