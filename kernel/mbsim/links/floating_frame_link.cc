/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/links/floating_frame_link.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  FloatingFrameLink::FloatingFrameLink(const std::string &name) : FrameLink(name), refFrame(nullptr), refFrameID(0), C("F") {
    C.setParent(this);
  }

  void FloatingFrameLink::resetUpToDate() {
    FrameLink::resetUpToDate();
    C.resetUpToDate();  
  }

  void FloatingFrameLink::calcSize() {
    ng = forceDir.cols() + momentDir.cols();
    ngd = ng;
    nla = ng;
    updSize = false;
  }

  void FloatingFrameLink::calclaSize(int j) {
    FrameLink::calclaSize(j);
    laSize = forceDir.cols() + momentDir.cols();
  }

  void FloatingFrameLink::calcgSize(int j) {
    FrameLink::calcgSize(j);
    gSize = forceDir.cols() + momentDir.cols();
  }

  void FloatingFrameLink::calcgdSize(int j) {
    FrameLink::calcgdSize(j);
    gdSize = forceDir.cols() + momentDir.cols();
  }

  void FloatingFrameLink::calcrFactorSize(int j) {
    FrameLink::calcrFactorSize(j);
    rFactorSize = isSetValued() ? forceDir.cols() + momentDir.cols() : 0;
  }

  void FloatingFrameLink::calccorrSize(int j) {
    FrameLink::calccorrSize(j);
    corrSize = forceDir.cols() + momentDir.cols();
  }

  void FloatingFrameLink::updateW(int j) {
    W[j][0] += C.evalJacobianOfTranslation(j).T() * evalRF(0) + C.evalJacobianOfRotation(j).T() * evalRM(0);
    W[j][1] += frame[1]->evalJacobianOfTranslation(j).T() * evalRF(1) + frame[1]->evalJacobianOfRotation(j).T() * evalRM(1);
  }

  void FloatingFrameLink::updateh(int j) {
    h[j][0] += C.evalJacobianOfTranslation(j).T() * evalForce(0) + C.evalJacobianOfRotation(j).T() * evalMoment(0);
    h[j][1] += frame[1]->evalJacobianOfTranslation(j).T() * evalForce(1) + frame[1]->evalJacobianOfRotation(j).T() * evalMoment(1);
  }

  void FloatingFrameLink::updatePositions(Frame *frame_) {
    frame_->setPosition(frame[1]->evalPosition());
    frame_->setOrientation(frame[0]->evalOrientation());
  }

  void FloatingFrameLink::updateVelocities() {
    WvP0P1 = frame[1]->evalVelocity() - C.evalVelocity();
    WomK0K1 = frame[1]->getAngularVelocity() - C.getAngularVelocity();
    updVel = false;
  }

  void FloatingFrameLink::updateGeneralizedPositions() {
    rrel.set(iF, evalGlobalForceDirection().T() * evalGlobalRelativePosition());
    rrel.set(iM, evalGeneralizedRelativePositionOfRotation());
    updrrel = false;
  }

  void FloatingFrameLink::updateGeneralizedVelocities() {
    vrel.set(iF, evalGlobalForceDirection().T() * evalGlobalRelativeVelocity());
    vrel.set(iM, evalGlobalMomentDirection().T() * evalGlobalRelativeAngularVelocity());
    updvrel = false;
  }

  void FloatingFrameLink::updateForceDirections() {
    DF = refFrame->evalOrientation() * forceDir;
    DM = refFrame->getOrientation() * momentDir;
    updDF = false;
  }

  void FloatingFrameLink::updateg() {
    g(iF) = evalGeneralizedRelativePosition()(iF);
    g(iM) = getGeneralizedRelativePosition()(iM);;
  }

  void FloatingFrameLink::updategd() {
    gd(iF) = evalGeneralizedRelativeVelocity()(iF);
    gd(iM) = getGeneralizedRelativeVelocity()(iM);
  }

  void FloatingFrameLink::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      iF = RangeV(0, forceDir.cols() - 1);
      iM = RangeV(forceDir.cols(), getGeneralizedRelativePositionSize() - 1);
      lambdaF.resize(forceDir.cols());
      lambdaM.resize(momentDir.cols());
    }
    else if(stage==unknownStage) {
      refFrame = refFrameID ? frame[1] : frame[0];
      C.setFrameOfReference(frame[0]);
      P[0] = frame[0];
      P[1] = &C;
    }
    FrameLink::init(stage, config);
  }

  void FloatingFrameLink::initializeUsingXML(DOMElement *element) {
    FrameLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"frameOfReferenceID");
    if (e) refFrameID = E(e)->getText<int>()-1;
  }

}
