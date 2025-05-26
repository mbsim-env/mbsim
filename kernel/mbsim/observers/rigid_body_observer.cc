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
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  const PlotFeatureEnum energy;

  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, energy)

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, RigidBodyObserver)

  RigidBodyObserver::RigidBodyObserver(const std::string &name) : Observer(name), body(NULL), frameOfReference(NULL) {
  }

  void RigidBodyObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_body.empty())
        setRigidBody(getByPath<RigidBody>(saved_body));
      if(not body)
        throwError("Rigid body is not given!");
      if(not saved_frameOfReference.empty())
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      if(not frameOfReference)
        setFrameOfReference(ds->getFrameI());
      if(not saved_outputFrame.empty())
        setOutputFrame(getByPath<Frame>(saved_outputFrame));
      if(not outputFrame)
        setOutputFrame(ds->getFrameI());
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[force]) {
          addToPlot("weight",{"x","y","z"});
          if(getDynamicSystemSolver()->getInverseKinetics()) {
            for(int i=0; i<body->getJoint()->getNumberOfForces(); i++)
              addToPlot("joint force "+to_string(i),{"x","y","z"});
          }
        }
        if(plotFeature[moment]) {
          if(getDynamicSystemSolver()->getInverseKinetics()) {
            for(int i=0; i<body->getJoint()->getNumberOfForces(); i++)
              addToPlot("joint moment "+to_string(i),{"x","y","z"});
          }
        }
        if(plotFeature[position]) {
          addToPlot("position",{"x","y","z"});
          addToPlot("direction",{"x","y","z"});
        }
        if(plotFeature[velocity])
          addToPlot("momentum",{"x","y","z"});
        if(plotFeature[angularVelocity])
          addToPlot("angular momentum",{"x","y","z"});
        if(plotFeature[acceleration])
          addToPlot("derivative of momentum",{"x","y","z"});
        if(plotFeature[angularAcceleration])
          addToPlot("derivative of angular momentum",{"x","y","z"});
        if(plotFeature[energy]) {
          addToPlot("kinetic energy");
          addToPlot("potential energy");
          addToPlot("total energy");
        }
      }
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
      if((ombvForce or ombvMoment) and not getDynamicSystemSolver()->getInverseKinetics())
        throwError("(RigidBodyObserver::init()): inverse kinetics not enabled");
    }
    else
      Observer::init(stage, config);
  }

  void RigidBodyObserver::plot() {
    Vec3 rOS = body->getFrameC()->evalPosition();
    Vec3 vS = body->getFrameC()->evalVelocity();
    Vec3 aS = body->getFrameC()->evalAcceleration();
    SqrMat3 AIK = body->getFrameC()->getOrientation();
    Vec3 om = body->getFrameC()->getAngularVelocity();
    Vec3 psi = body->getFrameC()->getAngularAcceleration();
    Vec3 p = body->getMass()*vS;
    Vec3 rOR = frameOfReference->evalPosition();
    Vec3 rRS = rOS - rOR;
    Vec3 vR = frameOfReference->evalVelocity();
    Vec3 vRS = vS - vR;
    Mat3x3 WThetaS = AIK*body->getInertiaTensor()*AIK.T();
    Vec3 L = WThetaS*om + crossProduct(rRS,body->getMass()*vRS);
    Vec3 pd = body->getMass()*aS;
    Vec3 aR = frameOfReference->evalAcceleration();
    Vec3 aRS = aS - aR;
    Vec3 Ld = WThetaS*psi + crossProduct(om,WThetaS*om) + crossProduct(rRS,body->getMass()*aRS);
    double absom = nrm2(om);
    Vec3 dr, dir;
    if(absom>1e-8) {
      SqrMat A(4,NONINIT);
      A.set(RangeV(0,2),RangeV(0,2),tilde(om));
      A.set(RangeV(0,2),RangeV(3,3),-om);
      A.set(RangeV(3,3),RangeV(0,2),om.T());
      A(3,3) = 0;
      Vec b(4,NONINIT);
      b.set(RangeV(0,2),-vS);
      b(3) = 0;
      Vec x = slvLU(A,b);
      dr = x(RangeV(0,2));
      dir = om/absom;
    }
    Vec3 r = rOS + dr;
    if(plotFeature[plotRecursive]) {
      auto TWOut = outputFrame->evalOrientation();
      if(plotFeature[force]) {
        Element::plot(TWOut.T()*body->getMass()*ds->getMBSimEnvironment()->getAccelerationOfGravity());
        if(getDynamicSystemSolver()->getInverseKinetics()) {
          for(int i=0; i<body->getJoint()->getNumberOfForces(); i++)
            Element::plot(TWOut.T()*body->getJoint()->evalForce(i));
        }
      }
      if(plotFeature[moment]) {
        if(getDynamicSystemSolver()->getInverseKinetics()) {
          for(int i=0; i<body->getJoint()->getNumberOfForces(); i++)
            Element::plot(TWOut.T()*body->getJoint()->evalMoment(i));
        }
      }
      if(plotFeature[position]) {
        Element::plot(TWOut.T()*r);
        Element::plot(TWOut.T()*dir);
      }
      if(plotFeature[velocity])
        Element::plot(TWOut.T()*p);
      if(plotFeature[angularVelocity])
        Element::plot(TWOut.T()*L);
      if(plotFeature[acceleration])
        Element::plot(TWOut.T()*pd);
      if(plotFeature[angularAcceleration])
        Element::plot(TWOut.T()*Ld);
      if(plotFeature[energy]) {
        double T = 0.5*(body->getMass()*vS.T()*vS + om.T()*WThetaS*om);
        double V = -body->getMass()*ds->getMBSimEnvironment()->getAccelerationOfGravity().T()*rOS;
        Element::plot(T);
        Element::plot(V);
        Element::plot(T+V);
      }
    }
    if(plotFeature[openMBV]) {
      if(FWeight) {
        array<double,8> data;
        data[0] = getTime();
        Vec3 G = body->getMass()*ds->getMBSimEnvironment()->getAccelerationOfGravity();
        data[1] = rOS(0);
        data[2] = rOS(1);
        data[3] = rOS(2);
        data[4] = G(0);
        data[5] = G(1);
        data[6] = G(2);
        data[7] = ombvWeight->getColorRepresentation()?nrm2(G):1.0;
        FWeight->append(data);
      }
      if(ombvForce) {
        int off = ombvForce->getSideOfInteraction()==0?1:0;
        for(size_t i=0; i<FArrow.size(); i++) {
          array<double,8> data;
          data[0] = getTime();
          Vec3 toPoint=body->getJoint()->getPointOfApplication(off+i)->evalPosition();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          Vec3 F = body->getJoint()->evalForce(off+i);
          data[4] = F(0);
          data[5] = F(1);
          data[6] = F(2);
          data[7] = ombvForce->getColorRepresentation()?nrm2(F):1.0;
          FArrow[i]->append(data);
        }
      }
      if(ombvMoment) {
        int off = ombvMoment->getSideOfInteraction()==0?1:0;
        for(size_t i=0; i<MArrow.size(); i++) {
          array<double,8> data;
          data[0] = getTime();
          Vec3 toPoint=body->getJoint()->getPointOfApplication(off+i)->evalPosition();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          Vec3 M = body->getJoint()->evalMoment(off+i);
          data[4] = M(0);
          data[5] = M(1);
          data[6] = M(2);
          data[7] = ombvMoment->getColorRepresentation()?nrm2(M):1.0;
          MArrow[i]->append(data);
        }
      }
      if(openMBVAxisOfRotation) {
        array<double,8> data;
        data[0] = getTime();
        data[1] = r(0);
        data[2] = r(1);
        data[3] = r(2);
        data[4] = dir(0);
        data[5] = dir(1);
        data[6] = dir(2);
        data[7] = ombvAxisOfRotation->getColorRepresentation()?nrm2(dir):0;
        openMBVAxisOfRotation->append(data);
      }
      if(openMBVMomentum) {
        array<double,8> data;
        data[0] = getTime();
        data[1] = rOS(0);
        data[2] = rOS(1);
        data[3] = rOS(2);
        data[4] = p(0);
        data[5] = p(1);
        data[6] = p(2);
        data[7] = ombvMomentum->getColorRepresentation()?nrm2(p):1;
        openMBVMomentum->append(data);
      }
      if(openMBVAngularMomentum) {
        array<double,8> data;
        data[0] = getTime();
        data[1] = rOR(0);
        data[2] = rOR(1);
        data[3] = rOR(2);
        data[4] = L(0);
        data[5] = L(1);
        data[6] = L(2);
        data[7] = ombvAngularMomentum->getColorRepresentation()?nrm2(L):1;
        openMBVAngularMomentum->append(data);
      }
      if(openMBVDerivativeOfMomentum) {
        array<double,8> data;
        data[0] = getTime();
        data[1] = rOS(0);
        data[2] = rOS(1);
        data[3] = rOS(2);
        data[4] = pd(0);
        data[5] = pd(1);
        data[6] = pd(2);
        data[7] = ombvDerivativeOfMomentum->getColorRepresentation()?nrm2(pd):1;
        openMBVDerivativeOfMomentum->append(data);
      }
      if(openMBVDerivativeOfAngularMomentum) {
        array<double,8> data;
        data[0] = getTime();
        data[1] = rOR(0);
        data[2] = rOR(1);
        data[3] = rOR(2);
        data[4] = Ld(0);
        data[5] = Ld(1);
        data[6] = Ld(2);
        data[7] = ombvDerivativeOfAngularMomentum->getColorRepresentation()?nrm2(Ld):1;
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

    e=E(element)->getFirstElementChildNamed(MBSIM%"outputFrame");
    if(e) saved_outputFrame=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVWeight");
    if(e) {
      ombvWeight = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvWeight->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointForce");
    if(e) {
      ombvForce = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvForce->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVJointMoment");
    if(e) {
      ombvMoment = shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(0,1,1,OpenMBVArrow::toDoubleHead,OpenMBVArrow::toPoint));
      ombvMoment->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAxisOfRotation");
    if(e) {
      ombvAxisOfRotation = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::line,OpenMBVArrow::midPoint));
      ombvAxisOfRotation->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMomentum");
    if(e) {
      ombvMomentum = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvMomentum->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularMomentum");
    if(e) {
      ombvAngularMomentum = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::toDoubleHead,OpenMBVArrow::toPoint));
      ombvAngularMomentum->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVDerivativeOfMomentum");
    if(e) {
      ombvDerivativeOfMomentum = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombvDerivativeOfMomentum->initializeUsingXML(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVDerivativeOfAngularMomentum");
    if(e) {
      ombvDerivativeOfAngularMomentum = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::toDoubleHead,OpenMBVArrow::toPoint));
      ombvDerivativeOfAngularMomentum->initializeUsingXML(e);
    }
  }

}
