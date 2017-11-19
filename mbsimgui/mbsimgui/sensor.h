/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _SENSOR__H_
#define _SENSOR__H_

#include "signal_.h"

namespace MBSimGUI {

  class Sensor : public Signal {
  };

  class ObjectSensor : public Sensor {
    public:
      QString getType() const override { return "ObjectSensor"; }
  };

  class GeneralizedPositionSensor : public ObjectSensor {
    public:
      QString getType() const override { return "GeneralizedPositionSensor"; }
      ElementPropertyDialog* createPropertyDialog() override {return new GeneralizedPositionSensorPropertyDialog(this);}

  };

  class GeneralizedVelocitySensor : public ObjectSensor {
    public:
      QString getType() const override { return "GeneralizedVelocitySensor"; }
      ElementPropertyDialog* createPropertyDialog() override {return new GeneralizedVelocitySensorPropertyDialog(this);}
  };

  class FrameSensor : public Sensor {
    public:
      QString getType() const override { return "FrameSensor"; }
  };

  class PositionSensor : public FrameSensor {
    public:
      QString getType() const override { return "PositionSensor"; }
      ElementPropertyDialog* createPropertyDialog() override {return new PositionSensorPropertyDialog(this);}
  };

  class OrientationSensor : public FrameSensor {
    public:
      QString getType() const override { return "OrientationSensor"; }
      ElementPropertyDialog* createPropertyDialog() override {return new OrientationSensorPropertyDialog(this);}
  };

  class VelocitySensor : public FrameSensor {
    public:
      QString getType() const override { return "VelocitySensor"; }
      ElementPropertyDialog* createPropertyDialog() override {return new VelocitySensorPropertyDialog(this);}
  };

  class AngularVelocitySensor : public FrameSensor {
    public:
      QString getType() const override { return "AngularVelocitySensor"; }
      ElementPropertyDialog* createPropertyDialog() override {return new AngularVelocitySensorPropertyDialog(this);}
  };

  class FunctionSensor : public Sensor {
    public:
      QString getType() const override { return "FunctionSensor"; }
      ElementPropertyDialog* createPropertyDialog() override {return new FunctionSensorPropertyDialog(this);}
  };

}

#endif
