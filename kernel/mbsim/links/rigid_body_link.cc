/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/links/rigid_body_link.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  RigidBodyLink::RigidBodyLink(const string &name) : MechanicalLink(name),  support(nullptr) {
  }

  void RigidBodyLink::updatehRef(const Vec &hParent, int j) {
    RangeV K = RangeV(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
    for(unsigned i=0; i<body.size(); i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      h[j][i]>>hParent(I);
      h[j][body.size()+i]>>hParent(K);
    }
  } 

  void RigidBodyLink::updaterRef(const Vec &rParent, int j) {
    RangeV K = RangeV(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
    for(unsigned i=0; i<body.size(); i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      r[j][i]>>rParent(I);
      r[j][body.size()+i]>>rParent(K);
    }
  } 

  void RigidBodyLink::updateWRef(const Mat &WParent, int j) {
    RangeV K = RangeV(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
    RangeV J = RangeV(laInd,laInd+laSize-1);
    for(unsigned i=0; i<body.size(); i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      W[j][i]>>WParent(I,J);
      W[j][body.size()+i]>>WParent(K,J);
    }
  } 

  void RigidBodyLink::updateVRef(const Mat &VParent, int j) {
    RangeV K = RangeV(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
    RangeV J = RangeV(laInd,laInd+laSize-1);
    for(unsigned i=0; i<body.size(); i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      V[j][i]>>VParent(I,J);
      V[j][body.size()+i]>>VParent(K,J);
    }
  } 

  void RigidBodyLink::updatePositions() {
    for(unsigned i=0; i<body.size(); i++) {
      C[i].setPosition(body[i]->getFrameForKinematics()->evalPosition());
      C[i].setOrientation(support->evalOrientation());
    }
    updPos = false;
  }

  void RigidBodyLink::updateGeneralizedPositions() {
    rrel.init(0);
    for(unsigned i=0; i<body.size(); i++)
      rrel+=body[i]->evalGeneralizedPosition()*ratio[i];
    updrrel = false;
  }

  void RigidBodyLink::updateGeneralizedVelocities() {
    vrel.init(0);
    for(unsigned i=0; i<body.size(); i++)
      vrel+=body[i]->evalGeneralizedVelocity()*ratio[i];
    updvrel = false;
  }

  void RigidBodyLink::updateForceDirections() {
    for(unsigned i=0; i<body.size(); i++) {
      DF[i] = support->evalOrientation()*body[i]->evalPJT();
      DM[i] = support->getOrientation()*body[i]->getPJR();
    }
    updFD = false;
  }

  void RigidBodyLink::updateForce() {
    for(unsigned i=0; i<body.size(); i++)
      F[i] = evalGlobalForceDirection(i)*evalGeneralizedForce()(iF)*ratio[i];
    updF = false;
  }

  void RigidBodyLink::updateMoment() {
    for(unsigned i=0; i<body.size(); i++)
      M[i] = evalGlobalMomentDirection(i)*evalGeneralizedForce()(iM)*ratio[i];
    updM = false;
  }

  void RigidBodyLink::updateR() {
    for(unsigned i=0; i<body.size(); i++) {
      RF[i].set(RangeV(0,2), iF, evalGlobalForceDirection(i)*ratio[i]);
      RM[i].set(RangeV(0,2), iM, getGlobalMomentDirection(i)*ratio[i]);
    }
    updRMV = false;
  }

  void RigidBodyLink::updateh(int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)
        h[j][i]+=body[i]->evalJRel(j).T()*evalGeneralizedForce()*ratio[i];
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        h[j][i]+=body[i]->getFrameForKinematics()->evalJacobianOfTranslation(j).T()*evalForce(i)  + body[i]->getFrameForKinematics()->evalJacobianOfRotation(j).T()*evalMoment(i);
        h[j][body.size()+i]-=C[i].evalJacobianOfTranslation(j).T()*evalForce(i) + C[i].evalJacobianOfRotation(j).T()*evalMoment(i);
      }
    }
  }

  void RigidBodyLink::updateW(int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)  {
        W[j][i]+=body[i]->evalJRel(j).T()*ratio[i];
      }
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        W[j][i]+=body[i]->getFrameForKinematics()->evalJacobianOfTranslation(j).T()*evalRF(i) + body[i]->getFrameForKinematics()->evalJacobianOfRotation(j).T()*evalRM(i);
        W[j][body.size()+i]-=C[i].evalJacobianOfTranslation(j).T()*evalRF(i) + C[i].evalJacobianOfRotation(j).T()*evalRM(i);
      }
    }
  }

  void RigidBodyLink::updateg() {
    g = evalGeneralizedRelativePosition();
  }

  void RigidBodyLink::updategd() {
    gd = evalGeneralizedRelativeVelocity();
  }

  void RigidBodyLink::updatewb() {
    for(unsigned i=0; i<body.size(); i++)
      wb += body[i]->evaljRel()*ratio[i];
  }

  void RigidBodyLink::calcSize() {
    ng = body[0]->getGeneralizedPositionSize();
    ngd = body[0]->getGeneralizedVelocitySize();
    nla = ngd;
    updSize = false;
  }

  void RigidBodyLink::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(saved_supportFrame!="")
        setSupportFrame(getByPath<Frame>(saved_supportFrame));
    }
    else if(stage==preInit) {
      if(!support)
        support = body[0]->getFrameOfReference();
      iF = RangeV(0, body[0]->getGeneralizedVelocitySize()-1);
      iM = RangeV(0, body[0]->getGeneralizedVelocitySize()-1);
      P.resize(body.size());
      F.resize(body.size());
      M.resize(body.size());
      DF.resize(body.size(),Mat3xV(iF.size()));
      DM.resize(body.size(),Mat3xV(iM.size()));
      RF.resize(body.size(),Mat3xV(getGeneralizedRelativeVelocitySize()));
      RM.resize(body.size(),Mat3xV(ngd));
      if(isSetValued()) {
        g.resize(ng);
        gd.resize(ngd);
        la.resize(nla);
      }
      h[0].resize(2*body.size());
      h[1].resize(2*body.size());
      W[0].resize(2*body.size());
      W[1].resize(2*body.size());
      V[0].resize(2*body.size());
      V[1].resize(2*body.size());
      C.resize(body.size());
      for(unsigned int i=0; i<body.size(); i++) {
        stringstream s;
        s << "F" << i;
        C[i].setName(s.str());
        C[i].setParent(this);
        C[i].setFrameOfReference(support);
      }
    }
    else if(stage==unknownStage) {
      for(unsigned int i=0; i<body.size(); i++)
        P[i] = body[i]->getFrameForKinematics();
    }
    MechanicalLink::init(stage, config);
  }

  void RigidBodyLink::initializeUsingXML(DOMElement* element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"supportFrame");
    if(e) saved_supportFrame=E(e)->getAttribute("ref");
  }

  void RigidBodyLink::resetUpToDate() {
    MechanicalLink::resetUpToDate();
    updPos = true;
    updVel = true;
    updFD = true;
    for(auto & i : C)
      i.resetUpToDate();
  }

}
