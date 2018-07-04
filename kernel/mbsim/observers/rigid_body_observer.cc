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
#include "mbsim/observers/rigid_body_observer.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/links/joint.h"
#include "mbsim/environment.h"
#include "mbsim/utils/eps.h"
#include "mbsim/dynamic_system_solver.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, RigidBodyObserver)

  RigidBodyObserver::RigidBodyObserver(const std::string &name) : Observer(name), body(NULL), frameOfReference(NULL) {
  }

  void RigidBodyObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(saved_body!="")
        setRigidBody(getByPath<RigidBody>(saved_body));
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        if(ombvWeight) {
          FWeight=ombvWeight->createOpenMBV();
          FWeight->setName("Weight");
          getOpenMBVGrp()->addObject(FWeight);
        }
        if(ombvForce) {
          FArrow.resize(ombvForce->getSideOfInteraction()==2?2:1);
          for(size_t i=0; i<FArrow.size(); i++) {
            FArrow[i]=ombvForce->createOpenMBV();
            FArrow[i]->setName(string("JointForce")+(FArrow.size()>1?to_string(i):string("")));
            getOpenMBVGrp()->addObject(FArrow[i]);
          }
        }
        if(ombvMoment) {
          MArrow.resize(ombvMoment->getSideOfInteraction()==2?2:1);
          for(size_t i=0; i<MArrow.size(); i++) {
            MArrow[i]=ombvMoment->createOpenMBV();
            MArrow[i]->setName(string("JointMoment")+(MArrow.size()>1?to_string(i):string("")));
            getOpenMBVGrp()->addObject(MArrow[i]);
          }
        }
        if(ombvAxisOfRotation) {
          openMBVAxisOfRotation=ombvAxisOfRotation->createOpenMBV();
          openMBVAxisOfRotation->setName("AxisOfRotation");
          getOpenMBVGrp()->addObject(openMBVAxisOfRotation);
        }
        if(ombvMomentum) {
          openMBVMomentum=ombvMomentum->createOpenMBV();
          openMBVMomentum->setName("Momentum");
          getOpenMBVGrp()->addObject(openMBVMomentum);
        }
        if(ombvAngularMomentum) {
          openMBVAngularMomentum=ombvAngularMomentum->createOpenMBV();
          openMBVAngularMomentum->setName("AngularMomentum");
          getOpenMBVGrp()->addObject(openMBVAngularMomentum);
        }
        if(ombvDerivativeOfMomentum) {
          openMBVDerivativeOfMomentum=ombvDerivativeOfMomentum->createOpenMBV();
          openMBVDerivativeOfMomentum->setName("DerivativeOfMomentum");
          getOpenMBVGrp()->addObject(openMBVDerivativeOfMomentum);
        }
        if(ombvDerivativeOfAngularMomentum) {
          openMBVDerivativeOfAngularMomentum=ombvDerivativeOfAngularMomentum->createOpenMBV();
          openMBVDerivativeOfAngularMomentum->setName("DerivativeOfAngularMomentum");
          getOpenMBVGrp()->addObject(openMBVDerivativeOfAngularMomentum);
        }
      }
    }
    else if(stage==unknownStage) {
      Observer::init(stage, config);
      if((FArrow.size() or MArrow.size()) and not getDynamicSystemSolver()->getInverseKinetics())
        throwError("(RigidBodyObserver::init()): inverse kinetics not enabled");
    }
    else
      Observer::init(stage, config);
  }

  void RigidBodyObserver::plot() {
    if(plotFeature[openMBV]) {
      Vec3 rOS = body->getFrameC()->evalPosition();
      Vec3 vS = body->getFrameC()->evalVelocity();
      Vec3 aS = body->getFrameC()->evalAcceleration();
      SqrMat3 AIK = body->getFrameC()->getOrientation();
      Vec3 om = body->getFrameC()->getAngularVelocity();
      Vec3 psi = body->getFrameC()->getAngularAcceleration();
      if(FWeight) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 G = body->getMass()*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
        data.push_back(rOS(0));
        data.push_back(rOS(1));
        data.push_back(rOS(2));
        data.push_back(G(0));
        data.push_back(G(1));
        data.push_back(G(2));
        data.push_back(ombvWeight->getColorRepresentation()?nrm2(G):1.0);
        FWeight->append(data);
      }
      if(ombvForce) {
        int off = ombvForce->getSideOfInteraction()==0?1:0;
        for(size_t i=0; i<FArrow.size(); i++) {
          vector<double> data;
          data.push_back(getTime());
          Vec3 toPoint=body->getJoint()->getPointOfApplication(off+i)->evalPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          Vec3 F = body->getJoint()->evalForce(off+i);
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          data.push_back(ombvForce->getColorRepresentation()?nrm2(F):1.0);
          FArrow[i]->append(data);
        }
      }
      if(ombvMoment) {
        int off = ombvMoment->getSideOfInteraction()==0?1:0;
        for(size_t i=0; i<MArrow.size(); i++) {
          vector<double> data;
          data.push_back(getTime());
          Vec3 toPoint=body->getJoint()->getPointOfApplication(off+i)->evalPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          Vec3 M = body->getJoint()->evalMoment(off+i);
          data.push_back(M(0));
          data.push_back(M(1));
          data.push_back(M(2));
          data.push_back(ombvMoment->getColorRepresentation()?nrm2(M):1.0);
          MArrow[i]->append(data);
        }
      }
      if(openMBVAxisOfRotation) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 dr;
        double absom = nrm2(om);
        if(abs(om(2))>macheps) {
          dr(0) = -vS(1)/om(2);
          dr(1) = vS(0)/om(2);
        }
        else if(abs(om(1))>macheps) {
          dr(0) = vS(2)/om(1);
          dr(2) = -vS(0)/om(1);
        }
        else if(abs(om(0))>macheps) {
          dr(1) = -vS(2)/om(0);
          dr(2) = vS(1)/om(0);
        }
        else
          absom = 1;
        Vec3 r = rOS + dr;
        data.push_back(r(0));
        data.push_back(r(1));
        data.push_back(r(2));
        Vec3 dir = om/absom;
        data.push_back(dir(0));
        data.push_back(dir(1));
        data.push_back(dir(2));
        data.push_back(ombvAxisOfRotation->getColorRepresentation()?nrm2(dir):0.5);
        openMBVAxisOfRotation->append(data);
        //          plotVector.push_back(nrm2(dir));
      }
      if(openMBVMomentum) {
        Vec3 p = body->getMass()*vS;
        vector<double> data;
        data.push_back(getTime());
        data.push_back(rOS(0));
        data.push_back(rOS(1));
        data.push_back(rOS(2));
        data.push_back(p(0));
        data.push_back(p(1));
        data.push_back(p(2));
        data.push_back(ombvMomentum->getColorRepresentation()?nrm2(p):1);
        openMBVMomentum->append(data);
      }
      if(openMBVAngularMomentum) {
        Vec3 rOR = frameOfReference?frameOfReference->evalPosition():rOS;
        Vec3 rRS = rOS - rOR;
        Vec3 vR = frameOfReference?frameOfReference->evalVelocity():vS;
        Vec3 vRS = vS - vR;
        Mat3x3 WThetaS = AIK*body->getInertiaTensor()*AIK.T();
        Vec3 L = WThetaS*om + crossProduct(rRS,body->getMass()*vRS);
        vector<double> data;
        data.push_back(getTime());
        data.push_back(rOR(0));
        data.push_back(rOR(1));
        data.push_back(rOR(2));
        data.push_back(L(0));
        data.push_back(L(1));
        data.push_back(L(2));
        data.push_back(ombvAngularMomentum->getColorRepresentation()?nrm2(L):1);
        openMBVAngularMomentum->append(data);
      }
      if(openMBVDerivativeOfMomentum) {
        Vec3 pd = body->getMass()*aS;
        vector<double> data;
        data.push_back(getTime());
        data.push_back(rOS(0));
        data.push_back(rOS(1));
        data.push_back(rOS(2));
        data.push_back(pd(0));
        data.push_back(pd(1));
        data.push_back(pd(2));
        data.push_back(ombvDerivativeOfMomentum->getColorRepresentation()?nrm2(pd):1);
        openMBVDerivativeOfMomentum->append(data);
      }
      if(openMBVDerivativeOfAngularMomentum) {
        Vec3 rOR = frameOfReference?frameOfReference->evalPosition():rOS;
        Vec3 rRS = rOS - rOR;
        Vec3 aR = frameOfReference?frameOfReference->evalAcceleration():aS;
        Vec3 aRS = aS - aR;
        Mat3x3 WThetaS = AIK*body->getInertiaTensor()*AIK.T();
        Vec3 Ld = WThetaS*psi + crossProduct(om,WThetaS*om) + crossProduct(rRS,body->getMass()*aRS);
        vector<double> data;
        data.push_back(getTime());
        data.push_back(rOR(0));
        data.push_back(rOR(1));
        data.push_back(rOR(2));
        data.push_back(Ld(0));
        data.push_back(Ld(1));
        data.push_back(Ld(2));
        data.push_back(ombvDerivativeOfAngularMomentum->getColorRepresentation()?nrm2(Ld):1);
        openMBVDerivativeOfAngularMomentum->append(data);
      }
    }
    Observer::plot();
  }
  
  void RigidBodyObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);

    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBody");
    saved_body=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(e) saved_frameOfReference=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVWeight");
    if(e) {
      ombvWeight = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint));
      ombvWeight->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointForce");
    if(e) {
      ombvForce = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint));
      ombvForce->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointMoment");
    if(e) {
      ombvMoment = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint));
      ombvMoment->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAxisOfRotation");
    if(e) {
      ombvAxisOfRotation = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBV::Arrow::line,OpenMBV::Arrow::midPoint));
      ombvAxisOfRotation->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMomentum");
    if(e) {
      ombvMomentum = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint));
      ombvMomentum->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularMomentum");
    if(e) {
      ombvAngularMomentum = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint));
      ombvAngularMomentum->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVDerivatveOfMomentum");
    if(e) {
      ombvDerivativeOfMomentum = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint));
      ombvDerivativeOfMomentum->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVDerivativeOfAngularMomentum");
    if(e) {
      ombvDerivativeOfAngularMomentum = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint));
      ombvDerivativeOfAngularMomentum->initializeUsingXML(e);
    }
  }

}
