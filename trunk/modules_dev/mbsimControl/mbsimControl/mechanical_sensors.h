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
 * Contact: schneidm@users.berlios.de
 */

#ifndef _MECHANICAL_SENSORS_H_
#define _MECHANICAL_SENSORS_H_

#include "mbsimControl/sensor.h"

namespace MBSim {

  class Frame;
  class Body;

  class AbsolutCoordinateSensor : public Sensor {
    public:
      AbsolutCoordinateSensor(const std::string &name) : Sensor(name) {}
      std::string getType() const { return "AbsolutCoordinateSensor"; }
      void setFrame(Frame * frame_) {frame=frame_; }
      void setDirection(fmatvec::Vec direction_) {direction=direction_/nrm2(direction_); assert(direction.size()==3); signal.resize(1); }
    protected:
      Frame * frame;
      fmatvec::Vec direction;
  };

  class AbsolutPositionSensor : public AbsolutCoordinateSensor {
    public:
      AbsolutPositionSensor(const std::string &name) : AbsolutCoordinateSensor(name) {}
      std::string getType() const { return "AbsolutPositionSensor"; }
      fmatvec::Vec getSignal();
  };

  class AbsolutVelocitySensor : public AbsolutCoordinateSensor {
    public:
      AbsolutVelocitySensor(const std::string &name) : AbsolutCoordinateSensor(name) {}
      std::string getType() const { return "AbsolutVelocitySensor"; }
      fmatvec::Vec getSignal();
  };

  class AbsolutAngularPositionSensor : public AbsolutCoordinateSensor {
    public:
      AbsolutAngularPositionSensor(const std::string &name) : AbsolutCoordinateSensor(name) {}
      std::string getType() const { return "AbsolutAngularPositionSensor"; }
      fmatvec::Vec getSignal();

      void calcxSize() {xSize=signal.cols(); }
      void calcgSize() {gSize=signal.cols(); }
      void calcgdSize() {gdSize=signal.cols(); }
      void init() {g.resize(signal.cols()); gd.resize(signal.cols()); }
      void updateg(double t) {g=x; }
      void updategd(double t);
      void updatexd(double t) {xd=gd; }
      void updatedx(double t, double dt) {xd=gd*dt; }
  };

  class AbsolutAngularVelocitySensor : public AbsolutCoordinateSensor {
    public:
      AbsolutAngularVelocitySensor(const std::string &name) : AbsolutCoordinateSensor(name) {}
      std::string getType() const { return "AbsolutAngularVelocitySensor"; }
      fmatvec::Vec getSignal();
  };
  
  
  class RelativeCoordinateSensor : public Sensor {
    public:
      RelativeCoordinateSensor(const std::string &name) : Sensor(name) {}
      std::string getType() const { return "RelativeCoordinateSensor"; }
      void setReferenceFrame(Frame * refFrame_) {refFrame=refFrame_; }
      void setRelativeFrame(Frame * relFrame_) {relFrame=relFrame_; }
      void setDirection(fmatvec::Vec direction_) {direction=direction_/nrm2(direction_); assert(direction.size()==3); signal.resize(1); }
    protected:
      Frame * refFrame;
      Frame * relFrame;
      fmatvec::Vec direction;
  };
  
  class RelativePositionSensor : public RelativeCoordinateSensor {
    public:
      RelativePositionSensor(const std::string &name) : RelativeCoordinateSensor(name) {}
      std::string getType() const { return "RelativePositionSensor"; }
      fmatvec::Vec getSignal();
  };
  
  class RelativeVelocitySensor : public RelativeCoordinateSensor {
    public:
      RelativeVelocitySensor(const std::string &name) : RelativeCoordinateSensor(name) {}
      std::string getType() const { return "RelativeVelocitySensor"; }
      fmatvec::Vec getSignal();
  };
  
  class RelativeAngularPositionSensor : public RelativeCoordinateSensor {
    public:
      RelativeAngularPositionSensor(const std::string &name) : RelativeCoordinateSensor(name) {}
      std::string getType() const { return "RelativeAngularPositionSensor"; }
      fmatvec::Vec getSignal();
      
      void calcxSize() {xSize=signal.cols(); }
      void calcgSize() {gSize=signal.cols(); }
      void calcgdSize() {gdSize=signal.cols(); }
      void init() {g.resize(signal.cols()); gd.resize(signal.cols()); }
      void updateg(double t) {g=x; }
      void updategd(double t);
      void updatexd(double t) {xd=gd; }
      void updatedx(double t, double dt) {xd=gd*dt; }
  };
  
  class RelativeAngularVelocitySensor : public RelativeCoordinateSensor {
    public:
      RelativeAngularVelocitySensor(const std::string &name) : RelativeCoordinateSensor(name) {}
      std::string getType() const { return "RelativeAngularVelocitySensor"; }
      fmatvec::Vec getSignal();
  };


  class GeneralizedCoordinateSensor : public Sensor {
    public:
      GeneralizedCoordinateSensor(const std::string &name) : Sensor(name) {}
      std::string getType() const { return "GeneralizedCoordinateSensor"; }
      void setBody(Body * body_) {body=body_; }
      void setIndex(int index_) {index=index_; signal.resize(1); }
    protected:
      Body * body;
      int index;
  };

  class GeneralizedPositionSensor : public GeneralizedCoordinateSensor {
    public:
      GeneralizedPositionSensor(const std::string &name) : GeneralizedCoordinateSensor(name) {}
      std::string getType() const { return "GeneralizedPositionSensor"; }
      fmatvec::Vec getSignal();
  };

  class GeneralizedVelocitySensor : public GeneralizedCoordinateSensor {
    public:
      GeneralizedVelocitySensor(const std::string &name) : GeneralizedCoordinateSensor(name) {}
      std::string getType() const { return "GeneralizedVelocitySensor"; }
      fmatvec::Vec getSignal();
  };

}

#endif /* _MECHANICAL_SENSORS_H_ */
