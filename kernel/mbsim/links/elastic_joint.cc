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

  ElasticJoint::ElasticJoint(const string &name) : FloatingFrameLink(name) {
  }

  ElasticJoint::~ElasticJoint() {
    delete func;
  }

  void ElasticJoint::updateGeneralizedForces() {
    if(func) lambda = -(*func)(evalGeneralizedRelativePosition(),evalGeneralizedRelativeVelocity());
    updla = false;
  }

  void ElasticJoint::updatexd() {
    if(integrateGeneralizedRelativeVelocityOfRotation)
      xd = evalGeneralizedRelativeVelocity()(iM);
  }

  void ElasticJoint::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(momentDir.cols()==2) {
        if(fabs(momentDir(2,0))<=macheps and fabs(momentDir(2,1))<=macheps)
          iR = 2;
        else if(fabs(momentDir(1,0))<=macheps and fabs(momentDir(1,1))<=macheps)
          iR = 1;
        else if(fabs(momentDir(0,0))<=macheps and fabs(momentDir(0,1))<=macheps)
          iR = 0;
        else
          THROW_MBSIMERROR("Generalized relative velocity of rotation can not be calculated from state for the defined moment direction. Turn on of integration generalized relative velocity of rotation.");
      }
      else if(momentDir.cols()==1) {
        msg(Warn) << "Evaluation of generalized relative velocity of rotation may be wrong for spatial rotation. In this case turn on integration of generalized relative velocity of rotation." << endl;
        if(fabs(momentDir(0,0))<=macheps and fabs(momentDir(2,0))<=macheps)
          iR = 2;
        else if(fabs(momentDir(1,0))<=macheps and fabs(momentDir(2,0))<=macheps)
          iR = 1;
        else if(fabs(momentDir(0,0))<=macheps and fabs(momentDir(1,0))<=macheps)
          iR = 0;
        else
          THROW_MBSIMERROR("Generalized relative velocity of rotation can not be calculated from state for the defined moment direction. Turn on of integration generalized relative velocity of rotation.");
      }
      eR(iR) = 1;
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

  VecV ElasticJoint::evalGeneralizedRelativePositionOfRotation() {
    if(integrateGeneralizedRelativeVelocityOfRotation)
      return x;
    else
      return evalGlobalMomentDirection().T()*frame[0]->getOrientation()*evalGlobalRelativeAngle();
  }

  Vec3 ElasticJoint::evalGlobalRelativeAngle() {
    WphiK0K1 = crossProduct(eR,AK0K1.col(iR));
    WphiK0K1(iR) = -AK0K1(fmod(iR+1,3),fmod(iR+2,3));
    return WphiK0K1;
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"integrateGeneralizedRelativeVelocityOfRotation");
    if(e) setIntegrateGeneralizedRelativeVelocityOfRotation(E(e)->getText<bool>());
  }

}
