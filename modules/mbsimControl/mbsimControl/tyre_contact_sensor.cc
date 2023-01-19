/* Copyright (C) 2004-2022 MBSim Development Team
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
#include "mbsimControl/tyre_contact_sensor.h"
#include "mbsim/links/tyre_contact.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/constitutive_laws/tyre_model.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void TyreContactSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"tyreContact");
    contactString=E(e)->getAttribute("ref");
  }

  void TyreContactSensor::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if (not contactString.empty())
        setTyreContact(getByPath<TyreContact>(contactString));
      if(not contact)
        throwError("Tyre contact is not given!");
    }
    Sensor::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, TyreContactPositionSensor)

  void TyreContactPositionSensor::updateSignal() {
    s = contact->getContourFrame(i)->evalPosition();
    upds = false;
  }

  void TyreContactPositionSensor::initializeUsingXML(DOMElement * element) {
    TyreContactSensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"positionNumber");
    if(e) setPositionNumber(E(e)->getText<Index>()-1);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, TyreContactOrientationSensor)

  void TyreContactOrientationSensor::updateSignal() {
    const SqrMat3 &A = contact->getContourFrame(i)->evalOrientation();
    int k=0;
    for(int i=0; i<A.rows(); i++) {
      for(int j=0; j<A.cols(); j++)
        s(k++) = A(i,j);
    }
    upds = false;
  }

  void TyreContactOrientationSensor::initializeUsingXML(DOMElement * element) {
    TyreContactSensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"orientationNumber");
    if(e) setOrientationNumber(E(e)->getText<Index>()-1);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, TyreContactVelocitySensor)

  void TyreContactVelocitySensor::updateSignal() {
    s = contact->getContourFrame(i)->evalVelocity();
    upds = false;
  }

  void TyreContactVelocitySensor::initializeUsingXML(DOMElement * element) {
    TyreContactSensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"velocityNumber");
    if(e) setVelocityNumber(E(e)->getText<Index>()-1);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, TyreContactAngularVelocitySensor)

  void TyreContactAngularVelocitySensor::updateSignal() {
    s = contact->getContourFrame(i)->evalAngularVelocity();
    upds = false;
  }

  void TyreContactAngularVelocitySensor::initializeUsingXML(DOMElement * element) {
    TyreContactSensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"angularVelocityNumber");
    if(e) setAngularVelocityNumber(E(e)->getText<Index>()-1);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, TyreModelSensor)

  int TyreModelSensor::getSignalSize() const { return contact->getTyreModel()->getDataSize(); }

  void TyreModelSensor::updateSignal() {
    s = contact->getTyreModel()->getData();
    upds = false;
  }

}
