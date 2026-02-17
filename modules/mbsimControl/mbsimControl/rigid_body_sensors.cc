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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimControl/rigid_body_sensors.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/joint.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void RigidBodySensor::init(InitStage stage, const InitConfigSet &config) {
    ObjectSensor::init(stage, config);
    if(stage==resolveStringRef) {
      if(not dynamic_cast<RigidBody*>(object))
        throwError("(RigidBodySensor::init): object is not a rigid body!");
    }
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, RigidBodyJointForceSensor)

  void RigidBodyJointForceSensor::updateSignal() {
    s = static_cast<RigidBody*>(object)->getJoint()->evalForce(i);
    upds = false;
  }

  void RigidBodyJointForceSensor::init(InitStage stage, const InitConfigSet &config) {
    RigidBodySensor::init(stage, config);
    if(stage==unknownStage) {
      if(not getDynamicSystemSolver()->getInverseKinetics())
        throwError("(RigidBodyJointForceSensor::init()): inverse kinetics not enabled");
    }
  }

  void RigidBodyJointForceSensor::initializeUsingXML(DOMElement * element) {
    RigidBodySensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"jointForceNumber");
    if(e) setJointForceNumber(E(e)->getText<Index>()-1);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, RigidBodyJointMomentSensor)

  void RigidBodyJointMomentSensor::updateSignal() {
    s = static_cast<RigidBody*>(object)->getJoint()->evalMoment(i);
    upds = false;
  }

  void RigidBodyJointMomentSensor::init(InitStage stage, const InitConfigSet &config) {
    RigidBodySensor::init(stage, config);
    if(stage==unknownStage) {
      if(not getDynamicSystemSolver()->getInverseKinetics())
        throwError("(RigidBodyJointMomentSensor::init()): inverse kinetics not enabled");
    }
  }

  void RigidBodyJointMomentSensor::initializeUsingXML(DOMElement * element) {
    RigidBodySensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"jointMomentNumber");
    if(e) setJointMomentNumber(E(e)->getText<Index>()-1);
  }

}
