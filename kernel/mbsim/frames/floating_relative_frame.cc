/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/frames/floating_relative_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void FloatingRelativeFrame::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
    }
    Frame::init(stage, config);
  }

  void FloatingRelativeFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
    DOMElement *ec;
    ec=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(ec) setFrameOfReference(E(ec)->getAttribute("ref"));
    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativePosition");
    if(ec) setRelativePosition(getVec3(ec));
    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativeOrientation");
    if(ec) setRelativeOrientation(getSqrMat3(ec));
  }

  void FloatingRelativeFrame::updatePositions() {
    parent->updatePositions(this);
    WrRP = getPosition(false) - R->evalPosition();
    updPos = false;
  }

  void FloatingRelativeFrame::updateVelocities() {
    setAngularVelocity(R->evalAngularVelocity());
    setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), evalGlobalRelativePosition()));
    updVel = false;
  }

  void FloatingRelativeFrame::updateAccelerations() {
    setAngularAcceleration(R->evalAngularAcceleration());
    setAcceleration(R->getAcceleration() + crossProduct(R->getAngularAcceleration(), evalGlobalRelativePosition()) + crossProduct(R->evalAngularVelocity(), crossProduct(R->evalAngularVelocity(), evalGlobalRelativePosition())));
    updAcc = true;
  }

  void FloatingRelativeFrame::updateJacobians(int j) {
    setJacobianOfRotation(R->evalJacobianOfRotation(j),j);
    setJacobianOfTranslation(R->getJacobianOfTranslation(j) - tilde(evalGlobalRelativePosition())*R->getJacobianOfRotation(j),j);
    updJac[j] = false;
  }

  void FloatingRelativeFrame::updateGyroscopicAccelerations() {
    setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation());
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() + crossProduct(R->getGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition()) + crossProduct(R->evalAngularVelocity(),crossProduct(R->evalAngularVelocity(),evalGlobalRelativePosition())));
    updGA = false;
  }

}
