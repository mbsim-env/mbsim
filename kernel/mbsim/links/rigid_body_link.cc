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

  void RigidBodyLink::updatehRef(Vec &hParent, int j) {
    RangeV K = RangeV(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
    for(unsigned i=0; i<body.size(); i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      h[j][i].ref(hParent,K);
      h[j][body.size()+i].ref(hParent,I);
    }
  } 

  void RigidBodyLink::updaterRef(Vec &rParent, int j) {
    RangeV K = RangeV(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
    for(unsigned i=0; i<body.size(); i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      r[j][i].ref(rParent,K);
      r[j][body.size()+i].ref(rParent,I);
    }
  } 

  void RigidBodyLink::updateWRef(Mat &WParent, int j) {
    RangeV K = RangeV(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
    RangeV J = RangeV(laInd,laInd+laSize-1);
    for(unsigned i=0; i<body.size(); i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      W[j][i].ref(WParent,K,J);
      W[j][body.size()+i].ref(WParent,I,J);
    }
  } 

  void RigidBodyLink::updateVRef(Mat &VParent, int j) {
    RangeV K = RangeV(support->gethInd(j),support->gethInd(j)+support->gethSize(j)-1);
    RangeV J = RangeV(laInd,laInd+laSize-1);
    for(unsigned i=0; i<body.size(); i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      V[j][i].ref(VParent,K,J);
      V[j][body.size()+i].ref(VParent,I,J);
    }
  } 

  void RigidBodyLink::updatePositions(Frame *frame_) {
    for(size_t i=0; i<body.size(); i++) {
      if(&C[i]==frame_) {
        frame_->setPosition(body[i]->getFrameForKinematics()->evalPosition());
        frame_->setOrientation(support->evalOrientation());
        break;
      }
    }
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
      DF[i] = body[i]->getFrameOfReference()->evalOrientation()*body[i]->evalPJT();
      DM[i] = body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR();
    }
    updFD = false;
  }

  void RigidBodyLink::updateForce() {
    for(unsigned i=0; i<body.size(); i++) {
      F[body.size()+i] = evalGlobalForceDirection(i)*evalGeneralizedForce()(iF)*ratio[i];
      F[i] = -F[body.size()+i];
    }
    updF = false;
  }

  void RigidBodyLink::updateMoment() {
    for(unsigned i=0; i<body.size(); i++) {
      M[body.size()+i] = evalGlobalMomentDirection(i)*evalGeneralizedForce()(iM)*ratio[i];
      M[i] = -M[body.size()+i];
    }
    updM = false;
  }

  void RigidBodyLink::updateR() {
    for(unsigned i=0; i<body.size(); i++) {
      RF[body.size()+i].set(RangeV(0,2), iF, evalGlobalForceDirection(i)*ratio[i]);
      RM[body.size()+i].set(RangeV(0,2), iM, getGlobalMomentDirection(i)*ratio[i]);
      RF[i] = -RF[body.size()+i];
      RM[i] = -RM[body.size()+i];
    }
    updRMV = false;
  }

  void RigidBodyLink::updateh(int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)
        h[j][body.size()+i] += body[i]->evalJRel(j).T()*evalGeneralizedForce()*ratio[i];
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        h[j][i] += C[i].evalJacobianOfTranslation(j).T()*evalForce(i) + C[i].evalJacobianOfRotation(j).T()*evalMoment(i);
        h[j][body.size()+i] += body[i]->getFrameForKinematics()->evalJacobianOfTranslation(j).T()*evalForce(body.size()+i) + body[i]->getFrameForKinematics()->evalJacobianOfRotation(j).T()*evalMoment(body.size()+i);
      }
    }
  }

  void RigidBodyLink::updateW(int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)
        W[j][body.size()+i] += body[i]->evalJRel(j).T()*ratio[i];
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        W[j][i] += C[i].evalJacobianOfTranslation(j).T()*evalRF(i) + C[i].evalJacobianOfRotation(j).T()*evalRM(i);
        W[j][body.size()+i] += body[i]->getFrameForKinematics()->evalJacobianOfTranslation(j).T()*evalRF(body.size()+i) + body[i]->getFrameForKinematics()->evalJacobianOfRotation(j).T()*evalRM(body.size()+i);
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
      if(not saved_supportFrame.empty())
        setSupportFrame(getByPath<Frame>(saved_supportFrame));
    }
    else if(stage==preInit) {
      if(!support)
        support = body[0]->getFrameOfReference();
      iF = RangeV(0, body[0]->getGeneralizedVelocitySize()-1);
      iM = RangeV(0, body[0]->getGeneralizedVelocitySize()-1);
      P.resize(2*body.size());
      F.resize(2*body.size());
      M.resize(2*body.size());
      DF.resize(2*body.size(),Mat3xV(iF.size()));
      DM.resize(2*body.size(),Mat3xV(iM.size()));
      RF.resize(2*body.size(),Mat3xV(getGeneralizedRelativeVelocitySize()));
      RM.resize(2*body.size(),Mat3xV(ngd));
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
      for(unsigned int i=0; i<body.size(); i++) {
        P[i] = &C[i];
        P[i+body.size()] = body[i]->getFrameForKinematics();
        C[i].sethSize(support->gethSize());
        C[i].sethSize(support->gethSize(1),1);
        C[i].init(stage, config);
      }
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
