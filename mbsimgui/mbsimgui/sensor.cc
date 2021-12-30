/*
   MBSimGUI - A fronted for MBSim.
   Copyright (C) 2021 MBSim-Env

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

#include <config.h>
#include "sensor.h"
#include "objectfactory.h"

using namespace std;

namespace MBSimGUI {

  MBSIMGUI_REGOBJECTFACTORY(AccelerationSensor);
  MBSIMGUI_REGOBJECTFACTORY(AngularAccelerationSensor);
  MBSIMGUI_REGOBJECTFACTORY(AngularVelocitySensor);
  MBSIMGUI_REGOBJECTFACTORY(FunctionSensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedAccelerationSensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedContactForceSensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedForceSensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedPositionSensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedRelativeContactPositionSensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedRelativeContactVelocitySensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedRelativePositionSensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedRelativeVelocitySensor);
  MBSIMGUI_REGOBJECTFACTORY(GeneralizedVelocitySensor);
  MBSIMGUI_REGOBJECTFACTORY(MechanicalConstraintForceSensor);
  MBSIMGUI_REGOBJECTFACTORY(MechanicalConstraintMomentSensor);
  MBSIMGUI_REGOBJECTFACTORY(MechanicalLinkForceSensor);
  MBSIMGUI_REGOBJECTFACTORY(MechanicalLinkMomentSensor);
  MBSIMGUI_REGOBJECTFACTORY(RigidBodyJointForceSensor);
  MBSIMGUI_REGOBJECTFACTORY(RigidBodyJointMomentSensor);
  MBSIMGUI_REGOBJECTFACTORY(OrientationSensor);
  MBSIMGUI_REGOBJECTFACTORY(PositionSensor);
  MBSIMGUI_REGOBJECTFACTORY(VelocitySensor);

}
