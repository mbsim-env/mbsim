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

  void RigidBodyObserver::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_body!="")
        setRigidBody(getByPath<RigidBody>(saved_body));
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Observer::init(stage);
    }
    else if(stage==plotting) {
      Observer::init(stage);
      if(plotFeature[13464197197848110344ULL]==enabled) {
        if(FWeight) {
          FWeight->setName("Weight");
          getOpenMBVGrp()->addObject(FWeight);
        }
        if(FArrow) {
          FArrow->setName("JointForce");
          getOpenMBVGrp()->addObject(FArrow);
        }
        if(MArrow) {
          MArrow->setName("JointMoment");
          getOpenMBVGrp()->addObject(MArrow);
        }
        if(openMBVAxisOfRotation) {
          openMBVAxisOfRotation->setName("AxisOfRotation");
          getOpenMBVGrp()->addObject(openMBVAxisOfRotation);
        }
        if(openMBVMomentum) {
          openMBVMomentum->setName("Momentum");
          getOpenMBVGrp()->addObject(openMBVMomentum);
        }
        if(openMBVAngularMomentum) {
          openMBVAngularMomentum->setName("AngularMomentum");
          getOpenMBVGrp()->addObject(openMBVAngularMomentum);
        }
        if(openMBVDerivativeOfMomentum) {
          openMBVDerivativeOfMomentum->setName("DerivativeOfMomentum");
          getOpenMBVGrp()->addObject(openMBVDerivativeOfMomentum);
        }
        if(openMBVDerivativeOfAngularMomentum) {
          openMBVDerivativeOfAngularMomentum->setName("DerivativeOfAngularMomentum");
          getOpenMBVGrp()->addObject(openMBVDerivativeOfAngularMomentum);
        }
      }
    }
    else
      Observer::init(stage);
  }

  void RigidBodyObserver::plot() {
    if(plotFeature[13464197197848110344ULL]==enabled) {
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
        data.push_back(1.0);
        FWeight->append(data);
      }
      if(FArrow) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=body->getJoint()->getFrame(1)->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 F = body->getJoint()->evalForce();
        data.push_back(F(0));
        data.push_back(F(1));
        data.push_back(F(2));
        data.push_back(nrm2(F));
        FArrow->append(data);
      }
      if(MArrow) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=body->getJoint()->getFrame(1)->evalPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 M = body->getJoint()->evalMoment();
        data.push_back(M(0));
        data.push_back(M(1));
        data.push_back(M(2));
        data.push_back(nrm2(M));
        MArrow->append(data);
      }
      if(openMBVAxisOfRotation) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 dr;
        double absom = nrm2(om);
        if(abs(om(2))>macheps()) {
          dr(0) = -vS(1)/om(2);
          dr(1) = vS(0)/om(2);
        }
        else if(abs(om(1))>macheps()) {
          dr(0) = vS(2)/om(1);
          dr(2) = -vS(0)/om(1);
        }
        else if(abs(om(0))>macheps()) {
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
        data.push_back(0.5);
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
        data.push_back(1.0);
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
        data.push_back(1.0);
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
        data.push_back(1.0);
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
        data.push_back(1.0);
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
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      FWeight=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointForce");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      FArrow=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointMoment");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      MArrow=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAxisOfRotation");
    if(e) {
      OpenMBVArrow ombv("[0.5;1;1]",0,OpenMBV::Arrow::line,OpenMBV::Arrow::midPoint,1,1);
      openMBVAxisOfRotation=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMomentum");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      openMBVMomentum=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularMomentum");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      openMBVAngularMomentum=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVDerivatveOfMomentum");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      openMBVDerivativeOfMomentum=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVDerivativeOfAngularMomentum");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      openMBVDerivativeOfAngularMomentum=ombv.createOpenMBV(e);
    }
  }

}
