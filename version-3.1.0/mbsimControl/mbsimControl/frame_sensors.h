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
   * \brief AbsolutCoordinateSensor
   * \author Markus Schneider
   */
  class AbsolutCoordinateSensor : public Sensor {
    public:
      AbsolutCoordinateSensor(const std::string &name) : Sensor(name), frame(NULL), direction(), frameString("") {}
      std::string getType() const { return "AbsolutCoordinateSensor"; }
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setFrame(MBSim::Frame * frame_) {frame=frame_; }
      void setDirection(fmatvec::Mat direction_) {
        direction=direction_;
        for (int i=0; i<direction_.cols(); i++)
          direction.col(i)=direction.col(i)/nrm2(direction.col(i)); 
        assert(direction.rows()==3); 
      }
    protected:
      MBSim::Frame * frame;
      fmatvec::Mat direction;
      std::string frameString;
  };

  /*!
   * \brief AbsolutePositionSensor
   * \author Markus Schneider
   */
  class AbsolutePositionSensor : public AbsolutCoordinateSensor {
    public:
      AbsolutePositionSensor(const std::string &name) : AbsolutCoordinateSensor(name) {}
      std::string getType() const { return "AbsolutePositionSensor"; }
      fmatvec::Vec getSignal();
  };

  /*!
   * \brief AbsoluteVelocitySensor
   * \author Markus Schneider
   */
  class AbsoluteVelocitySensor : public AbsolutCoordinateSensor {
    public:
      AbsoluteVelocitySensor(const std::string &name) : AbsolutCoordinateSensor(name) {}
      std::string getType() const { return "AbsoluteVelocitySensor"; }
      fmatvec::Vec getSignal();
  };

  /*!
   * \brief AbsoluteAngularPositionSensor
   * \author Markus Schneider
   */
  class AbsoluteAngularPositionSensor : public AbsolutCoordinateSensor {
    public:
      AbsoluteAngularPositionSensor(const std::string &name) : AbsolutCoordinateSensor(name) {}
      std::string getType() const { return "AbsoluteAngularPositionSensor"; }
      fmatvec::Vec getSignal();

      void calcxSize() {xSize=direction.cols(); }
      void init(MBSim::InitStage stage) {
        if (stage==MBSim::resize) {
          AbsolutCoordinateSensor::init(stage);
          g.resize(direction.cols()); 
          gd.resize(direction.cols()); 
          x.resize(direction.cols());
        }
        else
          AbsolutCoordinateSensor::init(stage);
      }
      void updateg(double t) {g=x; }
      void updategd(double t);
      void updatexd(double t) {xd=gd; }
      void updatedx(double t, double dt) {xd=gd*dt; }
  };

  /*!
   * \brief AbsoluteAngularVelocitySensor
   * \author Markus Schneider
   */
  class AbsoluteAngularVelocitySensor : public AbsolutCoordinateSensor {
    public:
      AbsoluteAngularVelocitySensor(const std::string &name) : AbsolutCoordinateSensor(name) {}
      std::string getType() const { return "AbsoluteAngularVelocitySensor"; }
      fmatvec::Vec getSignal();
  };
  
  
  /*!
   * \brief RelativeCoordinateSensor
   * \author Markus Schneider
   */
  class RelativeCoordinateSensor : public Sensor {
    public:
      RelativeCoordinateSensor(const std::string &name) : Sensor(name), refFrame(NULL), relFrame(NULL), direction(), refFrameString(""), relFrameString("") {}
      std::string getType() const { return "RelativeCoordinateSensor"; }
      void initializeUsingXML(TiXmlElement *element);
      void init(MBSim::InitStage stage);
      void setReferenceFrame(MBSim::Frame * refFrame_) {refFrame=refFrame_; }
      void setRelativeFrame(MBSim::Frame * relFrame_) {relFrame=relFrame_; }
      void setDirection(fmatvec::Mat direction_) {
        direction=direction_;
        for (int i=0; i<direction_.cols(); i++)
          direction.col(i)=direction.col(i)/nrm2(direction.col(i)); 
        assert(direction.rows()==3); 
      }
    protected:
      MBSim::Frame * refFrame;
      MBSim::Frame * relFrame;
      fmatvec::Mat direction;
      std::string refFrameString, relFrameString;
  };
  
  /*!
   * \brief RelativePositionSensor
   * \author Markus Schneider
   */
  class RelativePositionSensor : public RelativeCoordinateSensor {
    public:
      RelativePositionSensor(const std::string &name) : RelativeCoordinateSensor(name) {}
      std::string getType() const { return "RelativePositionSensor"; }
      fmatvec::Vec getSignal();
  };
  
  /*!
   * \brief RelativeVelocitySensor
   * \author Markus Schneider
   */
  class RelativeVelocitySensor : public RelativeCoordinateSensor {
    public:
      RelativeVelocitySensor(const std::string &name) : RelativeCoordinateSensor(name) {}
      std::string getType() const { return "RelativeVelocitySensor"; }
      fmatvec::Vec getSignal();
  };
  
  /*!
   * \brief RelativeAngularPositionSensor
   * \author Markus Schneider
   */
  class RelativeAngularPositionSensor : public RelativeCoordinateSensor {
    public:
      RelativeAngularPositionSensor(const std::string &name) : RelativeCoordinateSensor(name) {}
      std::string getType() const { return "RelativeAngularPositionSensor"; }
      fmatvec::Vec getSignal();
      
      void calcxSize() {xSize=direction.cols(); }
      void init(MBSim::InitStage stage) {
        if (stage==MBSim::resize) {
          RelativeCoordinateSensor::init(stage);
          g.resize(direction.cols()); 
          gd.resize(direction.cols());
          x.resize(direction.cols()); 
        }
        else
          RelativeCoordinateSensor::init(stage);
      }
      void updateg(double t) {g=x; }
      void updategd(double t);
      void updatexd(double t) {xd=gd; }
      void updatedx(double t, double dt) {xd=gd*dt; }
  };
  
  /*!
   * \brief RelativeAngularVelocitySensor
   * \author Markus Schneider
   */
  class RelativeAngularVelocitySensor : public RelativeCoordinateSensor {
    public:
      RelativeAngularVelocitySensor(const std::string &name) : RelativeCoordinateSensor(name) {}
      std::string getType() const { return "RelativeAngularVelocitySensor"; }
      fmatvec::Vec getSignal();
  };

}

#endif /* _FRAME_SENSORS_H_ */
