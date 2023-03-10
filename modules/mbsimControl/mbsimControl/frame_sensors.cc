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
#include "mbsim/frames/frame.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  void FrameSensor::initializeUsingXML(DOMElement * element) {
    Sensor::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"frame");
    saved_frame=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"outputFrame");
    if(e) saved_outputFrame=E(e)->getAttribute("ref");
  }

  void FrameSensor::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_frame.empty())
        setFrame(getByPath<Frame>(saved_frame));
      if(not frame)
        throwError("Frame is not given!");
      if(not saved_outputFrame.empty())
        setOutputFrame(getByPath<Frame>(saved_outputFrame));
      if(not outputFrame)
        setOutputFrame(ds->getFrameI());
    }
    Sensor::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, PositionSensor)

  void PositionSensor::updateSignal() {
    auto TWOut = outputFrame->evalOrientation();
    s = TWOut.T() * frame->evalPosition();
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, OrientationSensor)

  void OrientationSensor::updateSignal() {
    const SqrMat3 &A = frame->evalOrientation();
    int k=0;
    for(int i=0; i<A.rows(); i++) {
      for(int j=0; j<A.cols(); j++)
        s(k++) = A(i,j);
    }
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, VelocitySensor)

  void VelocitySensor::updateSignal() {
    auto TWOut = outputFrame->evalOrientation();
    s = TWOut.T() * frame->evalVelocity();
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, AngularVelocitySensor)

  void AngularVelocitySensor::updateSignal() {
    auto TWOut = outputFrame->evalOrientation();
    s = TWOut.T() * frame->evalAngularVelocity();
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, AccelerationSensor)

  void AccelerationSensor::updateSignal() {
    auto TWOut = outputFrame->evalOrientation();
    s = TWOut.T() * frame->evalAcceleration();
    upds = false;
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, AngularAccelerationSensor)

  void AngularAccelerationSensor::updateSignal() {
    auto TWOut = outputFrame->evalOrientation();
    s = TWOut.T() * frame->evalAngularAcceleration();
    upds = false;
  }

}
