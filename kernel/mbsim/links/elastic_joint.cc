/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsim/links/elastic_joint.h"
#include "mbsim/objectfactory.h"
#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, ElasticJoint)

  ElasticJoint::ElasticJoint(const string &name) : FloatingFrameLink(name), func(nullptr) {
  }

  ElasticJoint::~ElasticJoint() {
    delete func;
  }

  void ElasticJoint::updateGeneralizedForces() {
    if(func) lambda = -(*func)(evalGeneralizedRelativePosition(),evalGeneralizedRelativeVelocity());
    updla = false;
  }

  void ElasticJoint::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(momentDir.cols()==3) {
        evalGeneralizedRelativePositonOfRotation_ = &ElasticJoint::evalGeneralizedRelativePositonOfRotationFromState;
        evalGlobalRelativeAngle = &ElasticJoint::evalRelativePhixyz;
      }
      else if(momentDir.cols()==2) {
        evalGeneralizedRelativePositonOfRotation_ = &ElasticJoint::evalGeneralizedRelativePositonOfRotationFromState;
        if(nrm2(momentDir.row(2))<=macheps)
          evalGlobalRelativeAngle = &ElasticJoint::evalRelativePhixy;
        else if(nrm2(momentDir.row(1))<=macheps)
          evalGlobalRelativeAngle = &ElasticJoint::evalRelativePhixz;
        else
          evalGlobalRelativeAngle = &ElasticJoint::evalRelativePhiyz;
      }
      else if(momentDir.cols()==1) {
        msg(Info) << ("(FloatingFrameLink::evalRelativePhi): Evaluation of relative angle not yet implemented for this moment direction. Use integration of relative angular velocity instead.") << endl;
        evalGeneralizedRelativePositonOfRotation_ = &ElasticJoint::evalGeneralizedRelativePositonOfRotationByIntegration;
      }
      else {
        evalGeneralizedRelativePositonOfRotation_ = &ElasticJoint::evalGeneralizedRelativePositonOfRotationFromState;
        evalGlobalRelativeAngle = &ElasticJoint::evalRelativePhi;
      }
    }
    else if(stage==unknownStage) {
      if(func and (func->getRetSize().first!=forceDir.cols()+momentDir.cols())) THROW_MBSIMERROR("Size of generalized forces does not match!");
    }
    FloatingFrameLink::init(stage, config);
    if(func) func->init(stage, config);
  }

  void ElasticJoint::setForceDirection(const Mat3xV &fd) {

    forceDir = fd;

    for (int i = 0; i < fd.cols(); i++)
      forceDir.set(i, forceDir.col(i) / nrm2(fd.col(i)));
  }

  void ElasticJoint::setMomentDirection(const Mat3xV &md) {

    momentDir = md;

    for (int i = 0; i < md.cols(); i++)
      momentDir.set(i, momentDir.col(i) / nrm2(md.col(i)));
  }

  void ElasticJoint::initializeUsingXML(DOMElement *element) {
    FloatingFrameLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"forceDirection");
    if(e) setForceDirection(E(e)->getText<Mat>(3,0));
    e = E(element)->getFirstElementChildNamed(MBSIM%"momentDirection");
    if(e) setMomentDirection(E(e)->getText<Mat>(3,0));
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    if(e) {
      auto *f=ObjectFactory::createAndInit<Function<VecV(VecV,VecV)> >(e->getFirstElementChild());
      setGeneralizedForceFunction(f);
    }
  }

  VecV ElasticJoint::evalGeneralizedRelativePositonOfRotationFromState() {
    return evalGlobalMomentDirection().T() * frame[0]->getOrientation()*(this->*evalGlobalRelativeAngle)();
  }

  Vec3 ElasticJoint::evalRelativePhixyz() {
    WphiK0K1(0) = -AK0K1(1,2);
    WphiK0K1(1) = AK0K1(0,2);
    WphiK0K1(2) = -AK0K1(0,1);
    return WphiK0K1;
  }

  Vec3 ElasticJoint::evalRelativePhixy() {
    WphiK0K1(0) = -AK0K1(1,2);
    WphiK0K1(1) = AK0K1(0,2);
    return WphiK0K1;
  }

  Vec3 ElasticJoint::evalRelativePhixz() {
    WphiK0K1(0) = AK0K1(2,1);
    WphiK0K1(2) = -AK0K1(0,1);
    return WphiK0K1;
  }

  Vec3 ElasticJoint::evalRelativePhiyz() {
    WphiK0K1(1) = -AK0K1(2,0);
    WphiK0K1(2) = AK0K1(1,0);
    return WphiK0K1;
  }


}
