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
#include "weight.h"
#include "mbsimPhysics/namespace.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/dynamic_system.h"
//#include "mbsim/functions/constant_function.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimPhysics {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMPHYSICS, Weight)

  Weight::Weight(const std::string &name) : MechanicalLink(name), C("F") {
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
    C.setParent(this);
  }

  Weight::~Weight() {
    delete fg;
  }

  void Weight::setGravityFunction(MBSim::Function<double(double)> *func) {
    fg=func;
    fg->setParent(this);
    fg->setName("Gravity");
  }

  void Weight::resetUpToDate() {
    MechanicalLink::resetUpToDate();
    C.resetUpToDate();
    updPos = true;
  }

  void Weight::updateGeneralizedPositions() {
    rrel(0) = frame->evalOrientation().col(1).T()*evalGlobalRelativePosition();
    updrrel = false;
  }

  void Weight::updatePositions(Frame *frame_) {
    frame_->setPosition(body->getFrameC()->evalPosition() - frame->evalOrientation().col(1)*(frame->evalOrientation().col(1).T()*evalGlobalRelativePosition()));
    frame_->setOrientation(frame->evalOrientation());
  }

  void Weight::updatePositions() {
    WrP0P1 = body->getFrameC()->evalPosition() - frame->evalPosition();
    updPos = false;
  }

  void Weight::updatelaF() {
    lambdaF(0)=-body->getMass()*(*fg)(evalGeneralizedRelativePosition()(0));
    updlaF = false;
  }

  void Weight::updateForce() {
    F[1] = frame->evalOrientation().col(1)*evalGeneralizedForce()(0);
    F[0] = -F[1];
    updF = false;
  }

  void Weight::updateh(int j) {
    h[j][0] += C.evalJacobianOfTranslation(j).T() * evalForce(0);
    h[j][1] += body->getFrameC()->evalJacobianOfTranslation(j).T() * evalForce(1);
  }

  void Weight::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_ref1.empty() and not saved_ref2.empty())
        connect(getByPath<Frame>(saved_ref1), getByPath<RigidBody>(saved_ref2));
      if(not frame) frame = static_cast<DynamicSystem*>(parent)->getFrameI();
      if(not body) throwError("Not all connections are given!");
//      if(not fg) setGravityFunction(new ConstantFunction<double(double)>(9.80665));
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
      C.setFrameOfReference(frame);
      P[0] = &C;
      P[1] = body->getFrameC();
    }
    MechanicalLink::init(stage,config);
    fg->init(stage, config);
  }

  void Weight::plot() {
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

  void Weight::initializeUsingXML(xercesc::DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"connect");
    saved_ref1 = E(e)->getAttribute("ref1");
    saved_ref2 = E(e)->getAttribute("ref2");
    e = E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"gravityFunction");
    setGravityFunction(ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMPHYSICS%"enableOpenMBV");
    if(e) {
      ombvArrow = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvArrow->initializeUsingXML(e);
    }
  }

  void Weight::connect(Frame *frame_, RigidBody* body_) {
    frame = frame_;
    body = body_;
  }

  void Weight::updatehRef(const Vec &hParent, int j) {
    RangeV I = RangeV(frame->gethInd(j),frame->gethInd(j)+frame->gethSize(j)-1);
    h[j][0]>>hParent(I);
    I = RangeV(body->gethInd(j),body->gethInd(j)+body->gethSize(j)-1);
    h[j][1]>>hParent(I);
  }

  void Weight::updatedhdqRef(const fmatvec::Mat& dhdqParent, int k) {
    throwError("Internal error");
  }

  void Weight::updatedhduRef(const fmatvec::SqrMat& dhduParent, int k) {
    throwError("Internal error");
  }

  void Weight::updatedhdtRef(const fmatvec::Vec& dhdtParent, int j) {
    throwError("Internal error");
  }

}
