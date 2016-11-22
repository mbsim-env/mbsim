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
#include "mbsim/observers/rigid_body_group_observer.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/frame.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  RigidBodyGroupObserver::RigidBodyGroupObserver(const std::string &name) : Observer(name), ref(0) {
  }

  void RigidBodyGroupObserver::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      Observer::init(stage);
      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(openMBV)==enabled) {
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
          if(openMBVAngularVelocity) {
            openMBVAngularVelocity->setName("AngularVelocity");
            getOpenMBVGrp()->addObject(openMBVAngularVelocity);
          }
          if(openMBVAngularAcceleration) {
            openMBVAngularAcceleration->setName("AngularAcceleration");
            getOpenMBVGrp()->addObject(openMBVAngularAcceleration);
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
    }
    else
      Observer::init(stage);
  }

  void RigidBodyGroupObserver::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      double m = 0;
      for(unsigned int i=0; i<body.size(); i++) {
        m += body[i]->getMass();
      }
      Vec3 mpos, mvel, macc;
      Vec3 &p = mvel;
      Vec3 &pd = macc;
      Vec3 L, Ld;
      for(unsigned int i=0; i<body.size(); i++) {
        mpos += body[i]->getMass()*body[i]->getFrame("C")->evalPosition();
        mvel += body[i]->getMass()*body[i]->getFrame("C")->evalVelocity();
        macc += body[i]->getMass()*body[i]->getFrame("C")->evalAcceleration();
      }
      Vec3 rOS = mpos/m;
      Vec3 vS = mvel/m;
      Vec3 aS = macc/m;
      Vec3 rOR = ref?ref->evalPosition():rOS;
      Vec3 vR = ref?ref->evalVelocity():vS;
      Vec3 aR = ref?ref->evalAcceleration():aS;
      for(unsigned int i=0; i<body.size(); i++) {
        SqrMat3 AIK = body[i]->getFrame("C")->getOrientation();
        Vec3 rRSi = body[i]->getFrame("C")->getPosition() - rOR;
        Vec3 vSi = body[i]->getFrame("C")->getVelocity() - vR;
        Vec3 aSi = body[i]->getFrame("C")->getAcceleration() - aR;
        Vec3 omi = body[i]->getFrame("C")->getAngularVelocity();
        Vec3 psii = body[i]->getFrame("C")->getAngularAcceleration();
        double mi = body[i]->getMass();
        //L += (AIK*body[i]->getInertiaTensor()*AIK.T() - body[i]->getMass()*tilde(rRi)*tilde(rRi))*body[i]->getFrame("C")->getAngularVelocity();
        Mat3x3 WThetaS = AIK*body[i]->getInertiaTensor()*AIK.T();
        L += WThetaS*omi + crossProduct(rRSi,mi*vSi);
        Ld += WThetaS*psii + crossProduct(omi,WThetaS*omi) + crossProduct(rRSi,mi*aSi);
      }
      Vec3 G = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVWeight) {
          vector<double> data;
          data.push_back(getTime());
          data.push_back(rOS(0));
          data.push_back(rOS(1));
          data.push_back(rOS(2));
          data.push_back(G(0));
          data.push_back(G(1));
          data.push_back(G(2));
          data.push_back(0.5);
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
          data.push_back(0.5);
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
          data.push_back(0.5);
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
          data.push_back(0.5);
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
          data.push_back(0.5);
          openMBVDerivativeOfAngularMomentum->append(data);
        }
      }
      Observer::plot();
    }
  }
}
