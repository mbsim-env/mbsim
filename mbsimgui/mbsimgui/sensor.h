/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2013 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _SENSOR__H_
#define _SENSOR__H_

#include "signal_.h"

namespace MBSimGUI {

  class Sensor : public Signal {
    MBSIMGUI_OBJECTFACTORY_CLASS(Sensor, Signal, MBSIMCONTROL%"Sensor", "Sensor");
  };

  class ObjectSensor : public Sensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(ObjectSensor, Sensor, MBSIMCONTROL%"ObjectSensor", "Object sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new ObjectSensorPropertyDialog(this); }
  };

  class GeneralizedPositionSensor : public ObjectSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedPositionSensor, ObjectSensor, MBSIMCONTROL%"GeneralizedPositionSensor", "Generalized position sensor");
  };

  class GeneralizedVelocitySensor : public ObjectSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedVelocitySensor, ObjectSensor, MBSIMCONTROL%"GeneralizedVelocitySensor", "Generalized velocity sensor");
  };

  class GeneralizedAccelerationSensor : public ObjectSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedAccelerationSensor, ObjectSensor, MBSIMCONTROL%"GeneralizedAccelerationSensor", "Generalized acceleration sensor");
  };

  class RigidBodyJointForceSensor : public ObjectSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(RigidBodyJointForceSensor, ObjectSensor, MBSIMCONTROL%"RigidBodyJointForceSensor", "Rigid body joint force sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new RigidBodyJointForceSensorPropertyDialog(this); }
  };

  class RigidBodyJointMomentSensor : public ObjectSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(RigidBodyJointMomentSensor, ObjectSensor, MBSIMCONTROL%"RigidBodyJointMomentSensor", "Rigid body joint moment sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new RigidBodyJointMomentSensorPropertyDialog(this); }
  };

  class LinkSensor : public Sensor {
    public:
      PropertyDialog* createPropertyDialog() override { return new LinkSensorPropertyDialog(this); }
  };

  class GeneralizedRelativePositionSensor : public LinkSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedRelativePositionSensor, LinkSensor, MBSIMCONTROL%"GeneralizedRelativePositionSensor", "Generalized relative position sensor");
  };

  class GeneralizedRelativeVelocitySensor : public LinkSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedRelativeVelocitySensor, LinkSensor, MBSIMCONTROL%"GeneralizedRelativeVelocitySensor", "Generalized relative velocity sensor");
  };

  class GeneralizedForceSensor : public LinkSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedForceSensor, LinkSensor, MBSIMCONTROL%"GeneralizedForceSensor", "Generalized force sensor");
  };

  class MechanicalLinkForceSensor : public LinkSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(MechanicalLinkForceSensor, LinkSensor, MBSIMCONTROL%"MechanicalLinkForceSensor", "Mechanical link force sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new MechanicalLinkForceSensorPropertyDialog(this); }
  };

  class MechanicalLinkMomentSensor : public LinkSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(MechanicalLinkMomentSensor, LinkSensor, MBSIMCONTROL%"MechanicalLinkMomentSensor", "Mechanical link moment sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new MechanicalLinkMomentSensorPropertyDialog(this); }
  };

  class ConstraintSensor : public Sensor {
    public:
      PropertyDialog* createPropertyDialog() override { return new ConstraintSensorPropertyDialog(this); }
  };

  class MechanicalConstraintForceSensor : public ConstraintSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(MechanicalConstraintForceSensor, ConstraintSensor, MBSIMCONTROL%"MechanicalConstraintForceSensor", "Mechanical constraint force sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new MechanicalConstraintForceSensorPropertyDialog(this); }
  };

  class MechanicalConstraintMomentSensor : public ConstraintSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(MechanicalConstraintMomentSensor, ConstraintSensor, MBSIMCONTROL%"MechanicalConstraintMomentSensor", "Mechanical constraint moment sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new MechanicalConstraintMomentSensorPropertyDialog(this); }
  };

  class FrameSensor : public Sensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(FrameSensor, Sensor, MBSIMCONTROL%"FrameSensor", "Frame sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new FrameSensorPropertyDialog(this); }
  };

  class PositionSensor : public FrameSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(PositionSensor, FrameSensor, MBSIMCONTROL%"PositionSensor", "Position sensor");
  };

  class OrientationSensor : public FrameSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(OrientationSensor, FrameSensor, MBSIMCONTROL%"OrientationSensor", "Orientation sensor");
  };

  class VelocitySensor : public FrameSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(VelocitySensor, FrameSensor, MBSIMCONTROL%"VelocitySensor", "Velocity sensor");
  };

  class AngularVelocitySensor : public FrameSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(AngularVelocitySensor, FrameSensor, MBSIMCONTROL%"AngularVelocitySensor", "Angular velocity sensor");
  };

  class AccelerationSensor : public FrameSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(AccelerationSensor, FrameSensor, MBSIMCONTROL%"AccelerationSensor", "Acceleration sensor");
  };

  class AngularAccelerationSensor : public FrameSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(AngularAccelerationSensor, FrameSensor, MBSIMCONTROL%"AngularAccelerationSensor", "Angular acceleration sensor");
  };

  class FunctionSensor : public Sensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(FunctionSensor, Sensor, MBSIMCONTROL%"FunctionSensor", "Function sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new FunctionSensorPropertyDialog(this); }
  };

  class ContactSensor : public Sensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(ContactSensor, Sensor, MBSIMCONTROL%"ContactSensor", "Contact sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new ContactSensorPropertyDialog(this); }
  };

  class GeneralizedRelativeContactPositionSensor : public ContactSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedRelativeContactPositionSensor, ContactSensor, MBSIMCONTROL%"GeneralizedRelativeContactPositionSensor", "Generalized relative contact position sensor");
  };

  class GeneralizedRelativeContactVelocitySensor : public ContactSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedRelativeContactVelocitySensor, ContactSensor, MBSIMCONTROL%"GeneralizedRelativeContactVelocitySensor", "Generalized relative contact velocity sensor");
  };

  class GeneralizedContactForceSensor : public ContactSensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(GeneralizedContactForceSensor, ContactSensor, MBSIMCONTROL%"GeneralizedContactForceSensor", "Generalized contact force sensor");
  };

  class StateMachineSensor : public Sensor {
    MBSIMGUI_OBJECTFACTORY_CLASS(StateMachineSensor, Sensor, MBSIMCONTROL%"StateMachineSensor", "State machine sensor");
    public:
      PropertyDialog* createPropertyDialog() override { return new StateMachineSensorPropertyDialog(this); }
  };

}

#endif
