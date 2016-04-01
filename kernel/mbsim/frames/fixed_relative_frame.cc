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
#include "mbsim/frames/fixed_relative_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void FixedRelativeFrame::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Frame::init(stage);
    }
    else
      Frame::init(stage);
  }

  void FixedRelativeFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
    DOMElement *ec=element->getFirstElementChild();
    ec=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(ec) setFrameOfReference(E(ec)->getAttribute("ref"));
    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativePosition");
    if(ec) setRelativePosition(getVec3(ec));
    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativeOrientation");
    if(ec) setRelativeOrientation(getSqrMat3(ec));
  }

  DOMElement* FixedRelativeFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
//     if(getFrameOfReference()) {
//        DOMElement *ele1 = new DOMElement( MBSIM%"frameOfReference" );
//        string str = string("../Frame[") + getFrameOfReference()->getName() + "]";
//        ele1->SetAttribute("ref", str);
//        ele0->LinkEndChild(ele1);
//      }
//     addElementText(ele0,MBSIM%"relativePosition",getGeneralizedRelativePosition());
//     addElementText(ele0,MBSIM%"relativeOrientation",getRelativeOrientation());
   return ele0;
  }

  void FixedRelativeFrame::updatePositions() {
    setOrientation(R->evalOrientation()*ARP);
    WrRP = R->getOrientation()*RrRP;
    setPosition(R->getPosition() + WrRP);
    updPos = false;
  }

  void FixedRelativeFrame::updateVelocities() {
    setAngularVelocity(R->evalAngularVelocity());
    setVelocity(R->getVelocity() + crossProduct(R->getAngularVelocity(), evalGlobalRelativePosition()));
    updVel = false;
  }

  void FixedRelativeFrame::updateAccelerations() {
    setAngularAcceleration(R->evalAngularAcceleration());
    setAcceleration(R->getAcceleration() + crossProduct(R->getAngularAcceleration(), evalGlobalRelativePosition()) + crossProduct(R->evalAngularVelocity(), crossProduct(R->evalAngularVelocity(), evalGlobalRelativePosition())));
    updAcc = false;
  }

  void FixedRelativeFrame::updateJacobians(int j) {
    setJacobianOfRotation(R->evalJacobianOfRotation(j),j);
    setJacobianOfTranslation(R->getJacobianOfTranslation(j) - tilde(evalGlobalRelativePosition())*R->getJacobianOfRotation(j),j);
    updJac[j] = false;
  }

  void FixedRelativeFrame::updateGyroscopicAccelerations() {
    setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation());
    setGyroscopicAccelerationOfTranslation(R->getGyroscopicAccelerationOfTranslation() + crossProduct(R->getGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition()) + crossProduct(R->evalAngularVelocity(),crossProduct(R->evalAngularVelocity(),evalGlobalRelativePosition())));
    updGA = false;
  }

}
