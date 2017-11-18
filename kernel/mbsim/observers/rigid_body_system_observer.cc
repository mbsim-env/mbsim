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
#include "mbsim/observers/rigid_body_system_observer.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, RigidBodySystemObserver)

  RigidBodySystemObserver::RigidBodySystemObserver(const std::string &name) : Observer(name), frameOfReference(nullptr) {
  }

  void RigidBodySystemObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      for(const auto & i : saved_body)
        body.push_back(getByPath<RigidBody>(i));
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        if(openMBVPosition) {
          openMBVPosition->setName("Position");
          getOpenMBVGrp()->addObject(openMBVPosition);
        }
        if(openMBVVelocity) {
          openMBVVelocity->setName("Velocity");
          getOpenMBVGrp()->addObject(openMBVVelocity);
        }
        if(openMBVAcceleration) {
          openMBVAcceleration->setName("Acceleration");
          getOpenMBVGrp()->addObject(openMBVAcceleration);
        }
        if(openMBVWeight) {
          openMBVWeight->setName("Weight");
          getOpenMBVGrp()->addObject(openMBVWeight);
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
      Observer::init(stage, config);
  }

  void RigidBodySystemObserver::plot() {
    if(plotFeature[openMBV]) {
      double m = 0;
      for(auto & i : body) {
        m += i->getMass();
      }
      Vec3 mpos, mvel, macc;
      Vec3 &p = mvel;
      Vec3 &pd = macc;
      Vec3 L, Ld;
      for(auto & i : body) {
        mpos += i->getMass()*i->getFrameC()->evalPosition();
        mvel += i->getMass()*i->getFrameC()->evalVelocity();
        macc += i->getMass()*i->getFrameC()->evalAcceleration();
      }
      Vec3 rOS = mpos/m;
      Vec3 vS = mvel/m;
      Vec3 aS = macc/m;
      Vec3 rOR = frameOfReference?frameOfReference->evalPosition():rOS;
      Vec3 vR = frameOfReference?frameOfReference->evalVelocity():vS;
      Vec3 aR = frameOfReference?frameOfReference->evalAcceleration():aS;
      for(auto & i : body) {
        SqrMat3 AIK = i->getFrameC()->getOrientation();
        Vec3 rRSi = i->getFrameC()->getPosition() - rOR;
        Vec3 vRSi = i->getFrameC()->getVelocity() - vR;
        Vec3 aRSi = i->getFrameC()->getAcceleration() - aR;
        Vec3 omi = i->getFrameC()->getAngularVelocity();
        Vec3 psii = i->getFrameC()->getAngularAcceleration();
        double mi = i->getMass();
        Mat3x3 WThetaS = AIK*i->getInertiaTensor()*AIK.T();
        L += WThetaS*omi + crossProduct(rRSi,mi*vRSi);
        Ld += WThetaS*psii + crossProduct(omi,WThetaS*omi) + crossProduct(rRSi,mi*aRSi);
      }
      Vec3 G = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(rOR(0));
        data.push_back(rOR(1));
        data.push_back(rOR(2));
        data.push_back(0.5);
        openMBVPosition->append(data);
      }
      if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(rOR(0));
        data.push_back(rOR(1));
        data.push_back(rOR(2));
        data.push_back(vR(0));
        data.push_back(vR(1));
        data.push_back(vR(2));
        data.push_back(0.5);
        openMBVVelocity->append(data);
      }
      if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(rOR(0));
        data.push_back(rOR(1));
        data.push_back(rOR(2));
        data.push_back(aR(0));
        data.push_back(aR(1));
        data.push_back(aR(2));
        data.push_back(0.5);
        openMBVAcceleration->append(data);
      }
      if(openMBVWeight) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(rOS(0));
        data.push_back(rOS(1));
        data.push_back(rOS(2));
        data.push_back(G(0));
        data.push_back(G(1));
        data.push_back(G(2));
        data.push_back(1.0);
        openMBVWeight->append(data);
      }
      if(openMBVMomentum) {
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

  void RigidBodySystemObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);

    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBody");
    while(e && E(e)->getTagName()==MBSIM%"rigidBody") {
      saved_body.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(e) saved_frameOfReference=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVPosition");
    if(e) {
      OpenMBVArrow ombv;
      ombv.initializeUsingXML(e);
      openMBVPosition=ombv.createOpenMBV();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVVelocity");
    if(e) {
      OpenMBVArrow ombv;
      ombv.initializeUsingXML(e);
      openMBVVelocity=ombv.createOpenMBV();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAcceleration");
    if(e) {
      OpenMBVArrow ombv;
      ombv.initializeUsingXML(e);
      openMBVAcceleration=ombv.createOpenMBV();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVWeight");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      ombv.initializeUsingXML(e);
      openMBVWeight=ombv.createOpenMBV();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMomentum");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      ombv.initializeUsingXML(e);
      openMBVMomentum=ombv.createOpenMBV();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVAngularMomentum");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      ombv.initializeUsingXML(e);
      openMBVAngularMomentum=ombv.createOpenMBV();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVDerivatveOfMomentum");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      ombv.initializeUsingXML(e);
      openMBVDerivativeOfMomentum=ombv.createOpenMBV();
    }

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVDerivativeOfAngularMomentum");
    if(e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      ombv.initializeUsingXML(e);
      openMBVDerivativeOfAngularMomentum=ombv.createOpenMBV();
    }
  }

}
