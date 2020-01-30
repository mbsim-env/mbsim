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
#include "mbsimControl/constraint_sensors.h"
#include "mbsim/constraints/mechanical_constraint.h"
#include "mbsim/links/mechanical_link.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void ConstraintSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"constraint");
    constraintString=E(e)->getAttribute("ref");
  }

  void ConstraintSensor::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if (not constraintString.empty())
        setConstraint(getByPath<Constraint>(constraintString));
      if(not constraint)
        throwError("Constraint is not given!");
      Sensor::init(stage, config);
    }
    else
      Sensor::init(stage, config);
  }

  void MechanicalConstraintSensor::init(InitStage stage, const InitConfigSet &config) {
    ConstraintSensor::init(stage, config);
    if(stage==resolveStringRef) {
      if(not dynamic_cast<MechanicalConstraint*>(constraint))
        throwError("Constraint is not a mechanichal constraint!");
    }
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, MechanicalConstraintForceSensor)

  void MechanicalConstraintForceSensor::updateSignal() {
    s = static_cast<MechanicalConstraint*>(constraint)->getMechanicalLink()->evalForce(i);
    upds = false;
  }

  void MechanicalConstraintForceSensor::initializeUsingXML(DOMElement * element) {
    MechanicalConstraintSensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"forceNumber");
    if(e) setForceNumber(E(e)->getText<Index>()-1);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, MechanicalConstraintMomentSensor)

  void MechanicalConstraintMomentSensor::updateSignal() {
    s = static_cast<MechanicalConstraint*>(constraint)->getMechanicalLink()->evalMoment(i);
    upds = false;
  }

  void MechanicalConstraintMomentSensor::initializeUsingXML(DOMElement * element) {
    MechanicalConstraintSensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"momentNumber");
    if(e) setMomentNumber(E(e)->getText<Index>()-1);
  }

}
