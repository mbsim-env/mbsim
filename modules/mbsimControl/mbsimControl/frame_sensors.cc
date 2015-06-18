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

#include <config.h>
#include "mbsimControl/frame_sensors.h"

#include "mbsim/frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void AbsolutCoordinateSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"frame");
    frameString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"direction");
    direction=getMat(e,3,0);
    for (int i=0; i<direction.cols(); i++)
      direction.col(i)=direction.col(i)/nrm2(direction.col(i));
  }

  void AbsolutCoordinateSensor::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (frameString!="")
        setFrame(getByPath<Frame>(frameString));
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AbsolutePositionSensor, MBSIMCONTROL%"AbsolutePositionSensor")

  void AbsolutePositionSensor::updateSignal(double t) {
    s = direction.T()*frame->getPosition(t);
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AbsoluteVelocitySensor, MBSIMCONTROL%"AbsoluteVelocitySensor")

  void AbsoluteVelocitySensor::updateSignal(double t) {
    s = direction.T()*frame->getVelocity(t);
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AbsoluteAngularPositionSensor, MBSIMCONTROL%"AbsoluteAngularPositionSensor")

  void AbsoluteAngularPositionSensor::updatexd(double t) {
    xd=direction.T()*frame->getAngularVelocity(t);
  }

  void AbsoluteAngularPositionSensor::updatedx(double t, double dt) {
    xd=direction.T()*frame->getAngularVelocity(t)*dt;
  }

  void AbsoluteAngularPositionSensor::updateSignal(double t) {
    s = x;
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(AbsoluteAngularVelocitySensor, MBSIMCONTROL%"AbsoluteAngularVelocitySensor")

  void AbsoluteAngularVelocitySensor::updateSignal(double t) {
    s = direction.T()*frame->getAngularVelocity(t);
    upds = false;
  }

  void RelativeCoordinateSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"frame");
    refFrameString=E(e)->getAttribute("ref");
    relFrameString=E(e)->getAttribute("rel");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"direction");
    direction=getMat(e,3,0);
    for (int i=0; i<direction.cols(); i++)
      direction.col(i)=direction.col(i)/nrm2(direction.col(i));
  }

  void RelativeCoordinateSensor::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if (refFrameString!="")
        setReferenceFrame(getByPath<Frame>(refFrameString));
      if (relFrameString!="")
        setRelativeFrame(getByPath<Frame>(relFrameString));
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(RelativePositionSensor, MBSIMCONTROL%"RelativePositionSensor")

  void RelativePositionSensor::updateSignal(double t) {
    VecV WrRefRel=relFrame->getPosition(t)-refFrame->getPosition(t);
    s = (refFrame->getOrientation(t)*direction).T()*WrRefRel;
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(RelativeVelocitySensor, MBSIMCONTROL%"RelativeVelocitySensor")

  void RelativeVelocitySensor::updateSignal(double t) {
    VecV WvRefRel=relFrame->getVelocity(t)-refFrame->getVelocity(t);
    s = (refFrame->getOrientation(t)*direction).T()*WvRefRel;
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(RelativeAngularPositionSensor, MBSIMCONTROL%"RelativeAngularPositionSensor")

  void RelativeAngularPositionSensor::updatexd(double t) {
    VecV WomegaRefRel=relFrame->getAngularVelocity(t)-refFrame->getAngularVelocity(t);
    xd=(refFrame->getOrientation(t)*direction).T()*WomegaRefRel;
  }

  void RelativeAngularPositionSensor::updatedx(double t, double dt) {
    VecV WomegaRefRel=relFrame->getAngularVelocity(t)-refFrame->getAngularVelocity(t);
    xd=(refFrame->getOrientation(t)*direction).T()*WomegaRefRel*dt;
  }

  void RelativeAngularPositionSensor::updateSignal(double t) {
    s = x;
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(RelativeAngularVelocitySensor, MBSIMCONTROL%"RelativeAngularVelocitySensor")

  void RelativeAngularVelocitySensor::updateSignal(double t) {
    VecV WomegaRefRel=relFrame->getAngularVelocity(t)-refFrame->getAngularVelocity(t);
    s = (refFrame->getOrientation(t)*direction).T()*WomegaRefRel;
    upds = false;
  }

}
