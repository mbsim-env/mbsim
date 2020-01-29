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
      ElementPropertyDialog* createPropertyDialog() override { return new ObjectSensorPropertyDialog(this); }
  };

  class GeneralizedPositionSensor : public ObjectSensor {
    public:
      QString getType() const override { return "GeneralizedPositionSensor"; }
  };

  class GeneralizedVelocitySensor : public ObjectSensor {
    public:
      QString getType() const override { return "GeneralizedVelocitySensor"; }
  };

  class GeneralizedAccelerationSensor : public ObjectSensor {
    public:
      QString getType() const override { return "GeneralizedAccelerationSensor"; }
  };

  class LinkSensor : public Sensor {
    public:
      QString getType() const override { return "LinkSensor"; }
      ElementPropertyDialog* createPropertyDialog() override { return new LinkSensorPropertyDialog(this); }
  };

  class GeneralizedRelativePositionSensor : public LinkSensor {
    public:
      QString getType() const override { return "GeneralizedRelativePositionSensor"; }
  };

  class GeneralizedRelativeVelocitySensor : public LinkSensor {
    public:
      QString getType() const override { return "GeneralizedRelativeVelocitySensor"; }
  };

  class GeneralizedForceSensor : public LinkSensor {
    public:
      QString getType() const override { return "GeneralizedForceSensor"; }
  };

  class FrameSensor : public Sensor {
    public:
      QString getType() const override { return "FrameSensor"; }
      ElementPropertyDialog* createPropertyDialog() override { return new FrameSensorPropertyDialog(this); }
  };

  class PositionSensor : public FrameSensor {
    public:
      QString getType() const override { return "PositionSensor"; }
  };

  class OrientationSensor : public FrameSensor {
    public:
      QString getType() const override { return "OrientationSensor"; }
  };

  class VelocitySensor : public FrameSensor {
    public:
      QString getType() const override { return "VelocitySensor"; }
  };

  class AngularVelocitySensor : public FrameSensor {
    public:
      QString getType() const override { return "AngularVelocitySensor"; }
  };

  class AccelerationSensor : public FrameSensor {
    public:
      QString getType() const override { return "AccelerationSensor"; }
  };

  class AngularAccelerationSensor : public FrameSensor {
    public:
      QString getType() const override { return "AngularAccelerationSensor"; }
  };

  class FunctionSensor : public Sensor {
    public:
      QString getType() const override { return "FunctionSensor"; }
      ElementPropertyDialog* createPropertyDialog() override { return new FunctionSensorPropertyDialog(this); }
  };

  class ContactSensor : public Sensor {
    public:
      QString getType() const override { return "ContactSensor"; }
      ElementPropertyDialog* createPropertyDialog() override { return new ContactSensorPropertyDialog(this); }
  };

  class GeneralizedRelativeContactPositionSensor : public ContactSensor {
    public:
      QString getType() const override { return "GeneralizedRelativeContactPositionSensor"; }

  };

  class GeneralizedRelativeContactVelocitySensor : public ContactSensor {
    public:
      QString getType() const override { return "GeneralizedRelativeContactVelocitySensor"; }
  };

  class GeneralizedContactForceSensor : public ContactSensor {
    public:
      QString getType() const override { return "GeneralizedContactForceSensor"; }
  };

}

#endif
