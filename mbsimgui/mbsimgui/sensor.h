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
      PropertyDialog* createPropertyDialog() override { return new ObjectSensorPropertyDialog(this); }
  };

  class GeneralizedPositionSensor : public ObjectSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedPositionSensor"; }
      QString getType() const override { return "Generalized position sensor"; }
  };

  class GeneralizedVelocitySensor : public ObjectSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedVelocitySensor"; }
      QString getType() const override { return "Generalized velocity sensor"; }
  };

  class GeneralizedAccelerationSensor : public ObjectSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedAccelerationSensor"; }
      QString getType() const override { return "Generalized acceleration sensor"; }
  };

  class RigidBodyJointForceSensor : public ObjectSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"RigidBodyJointForceSensor"; }
      QString getType() const override { return "Rigid body joint force sensor"; }
      PropertyDialog* createPropertyDialog() override { return new RigidBodyJointForceSensorPropertyDialog(this); }
  };

  class RigidBodyJointMomentSensor : public ObjectSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"RigidBodyJointMomentSensor"; }
      QString getType() const override { return "Rigid body joint moment sensor"; }
      PropertyDialog* createPropertyDialog() override { return new RigidBodyJointMomentSensorPropertyDialog(this); }
  };

  class LinkSensor : public Sensor {
    public:
      PropertyDialog* createPropertyDialog() override { return new LinkSensorPropertyDialog(this); }
  };

  class GeneralizedRelativePositionSensor : public LinkSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedRelativePositionSensor"; }
      QString getType() const override { return "Generalized relative position sensor"; }
  };

  class GeneralizedRelativeVelocitySensor : public LinkSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedRelativeVelocitySensor"; }
      QString getType() const override { return "Generalized relative velocity sensor"; }
  };

  class GeneralizedForceSensor : public LinkSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedForceSensor"; }
      QString getType() const override { return "Generalized force sensor"; }
  };

  class MechanicalLinkForceSensor : public LinkSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"MechanicalLinkForceSensor"; }
      QString getType() const override { return "Mechanical link force sensor"; }
      PropertyDialog* createPropertyDialog() override { return new MechanicalLinkForceSensorPropertyDialog(this); }
  };

  class MechanicalLinkMomentSensor : public LinkSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"MechanicalLinkMomentSensor"; }
      QString getType() const override { return "Mechanical link moment sensor"; }
      PropertyDialog* createPropertyDialog() override { return new MechanicalLinkMomentSensorPropertyDialog(this); }
  };

  class ConstraintSensor : public Sensor {
    public:
      PropertyDialog* createPropertyDialog() override { return new ConstraintSensorPropertyDialog(this); }
  };

  class MechanicalConstraintForceSensor : public ConstraintSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"MechanicalConstraintForceSensor"; }
      QString getType() const override { return "Mechanical constraint force sensor"; }
      PropertyDialog* createPropertyDialog() override { return new MechanicalConstraintForceSensorPropertyDialog(this); }
  };

  class MechanicalConstraintMomentSensor : public ConstraintSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"MechanicalConstraintMomentSensor"; }
      QString getType() const override { return "Mechanical constraint moment sensor"; }
      PropertyDialog* createPropertyDialog() override { return new MechanicalConstraintMomentSensorPropertyDialog(this); }
  };

  class FrameSensor : public Sensor {
    public:
      PropertyDialog* createPropertyDialog() override { return new FrameSensorPropertyDialog(this); }
  };

  class PositionSensor : public FrameSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"PositionSensor"; }
      QString getType() const override { return "Position sensor"; }
  };

  class OrientationSensor : public FrameSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"OrientationSensor"; }
      QString getType() const override { return "Orientation sensor"; }
  };

  class VelocitySensor : public FrameSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"VelocitySensor"; }
      QString getType() const override { return "Velocity sensor"; }
  };

  class AngularVelocitySensor : public FrameSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"AngularVelocitySensor"; }
      QString getType() const override { return "Angular velocity sensor"; }
  };

  class AccelerationSensor : public FrameSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"AccelerationSensor"; }
      QString getType() const override { return "Acceleration sensor"; }
  };

  class AngularAccelerationSensor : public FrameSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"AngularAccelerationSensor"; }
      QString getType() const override { return "Angular acceleration sensor"; }
  };

  class FunctionSensor : public Sensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"FunctionSensor"; }
      QString getType() const override { return "Function sensor"; }
      PropertyDialog* createPropertyDialog() override { return new FunctionSensorPropertyDialog(this); }
  };

  class ContactSensor : public Sensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"ContactSensor"; }
      QString getType() const override { return "Contact sensor"; }
      PropertyDialog* createPropertyDialog() override { return new ContactSensorPropertyDialog(this); }
  };

  class GeneralizedRelativeContactPositionSensor : public ContactSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedRelativeContactPositionSensor"; }
      QString getType() const override { return "Generalized relative contact position sensor"; }

  };

  class GeneralizedRelativeContactVelocitySensor : public ContactSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedRelativeContactVelocitySensor"; }
      QString getType() const override { return "Generalized relative contact velocity sensor"; }
  };

  class GeneralizedContactForceSensor : public ContactSensor {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"GeneralizedContactForceSensor"; }
      QString getType() const override { return "Generalized contact force sensor"; }
  };

}

#endif
