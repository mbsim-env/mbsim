/* Copyright (C) 2004-2018 MBSim Development Team
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
#include "universal_gravitation.h"
#include "mbsimPhysics/namespace.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimPhysics {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMPHYSICS, UniversalGravitation)

  UniversalGravitation::UniversalGravitation(const std::string &name) : MechanicalLink(name), body(2) {
    nla = 1;
    ng = 1;
    ngd = 1;
    lambdaF.resize(1);
    iF = RangeV(0, 0);
    iM = RangeV(0, -1);
    P.resize(2);
    F.resize(2);
    for(unsigned int i=0; i<2; i++)
      h[i].resize(2);
  }

  void UniversalGravitation::resetUpToDate() {
    MechanicalLink::resetUpToDate();
    updPos = true;
  }

  void UniversalGravitation::updateGeneralizedPositions() {
    rrel(0) = nrm2(evalGlobalRelativePosition());
    updrrel = false;
  }

  void UniversalGravitation::updatePositions() {
    WrP0P1 = body[1]->getFrameC()->evalPosition() - body[0]->getFrameC()->evalPosition();
    updPos = false;
  }

  void UniversalGravitation::updatelaF() {
    lambdaF(0)=-ga*body[0]->getMass()*body[1]->getMass()/pow(evalGeneralizedRelativePosition()(0),2);
    updlaF = false;
  }

  void UniversalGravitation::updateForce() {
    F[1] = (evalGlobalRelativePosition()/evalGeneralizedRelativePosition()(0))*evalGeneralizedForce()(0);
    F[0] = -F[1];
    updF = false;
  }

  void UniversalGravitation::updateh(int j) {
    for(unsigned int i=0; i<h[j].size(); i++)
      h[j][i] += body[i]->getFrameC()->evalJacobianOfTranslation(j).T() * evalForce(i);
  }

  void UniversalGravitation::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_ref1.empty() and not saved_ref2.empty())
        connect(getByPath<RigidBody>(saved_ref1), getByPath<RigidBody>(saved_ref2));
      if(not body[0] or not body[1])
        throwError("Not all connections are given!");
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] and ombvArrow) {
        openMBVForce.resize(ombvArrow->getSideOfInteraction()==2?getNumberOfLinks():getNumberOfLinks()/2);
        for(size_t i=0; i<openMBVForce.size(); i++) {
          openMBVForce[i]=ombvArrow->createOpenMBV();
          openMBVForce[i]->setName(name+"_Force"+(openMBVForce.size()>1?to_string(i):string("")));
          parent->getOpenMBVGrp()->addObject(openMBVForce[i]);
        }
      }
    }
    else if(stage==unknownStage) {
      P[0] = body[0]->getFrameC();
      P[1] = body[1]->getFrameC();
    }
    MechanicalLink::init(stage,config);
  }

  void UniversalGravitation::plot() {
    if(plotFeature[openMBV] and ombvArrow) {
      int off = ombvArrow->getSideOfInteraction()==0?getNumberOfLinks()/2:0;
      for(size_t i=0; i<openMBVForce.size(); i++) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=getPointOfApplication(off+i)->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WF = evalForce(off+i);
        data.push_back(WF(0));
        data.push_back(WF(1));
        data.push_back(WF(2));
        data.push_back(ombvArrow->getColorRepresentation()?nrm2(evalForce()):1);
        openMBVForce[i]->append(data);
      }
    }
    MechanicalLink::plot();
  }

  void UniversalGravitation::initializeUsingXML(xercesc::DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"connect");
    saved_ref1 = E(e)->getAttribute("ref1");
    saved_ref2 = E(e)->getAttribute("ref2");
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"gravitationalConstant");
    if(e) setGravitationalConstant(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"enableOpenMBV");
    if(e) {
      ombvArrow = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvArrow->initializeUsingXML(e);
    }
  }

  void UniversalGravitation::connect(RigidBody *body0, RigidBody* body1) {
    body[0] = body0;
    body[1] = body1;
  }

  void UniversalGravitation::updatehRef(Vec &hParent, int j) {
    for(unsigned i=0; i<2; i++) {
      RangeV I = RangeV(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      h[j][i].ref(hParent,I);
    }
  }

  void UniversalGravitation::updatedhdqRef(fmatvec::Mat& dhdqParent, int k) {
    throwError("Internal error");
  }

  void UniversalGravitation::updatedhduRef(fmatvec::SqrMat& dhduParent, int k) {
    throwError("Internal error");
  }

  void UniversalGravitation::updatedhdtRef(fmatvec::Vec& dhdtParent, int j) {
    throwError("Internal error");
  }

}
