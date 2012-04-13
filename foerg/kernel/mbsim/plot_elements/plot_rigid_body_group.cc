/* Copyright (C) 2004-2011 MBSim Development Team
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
#include "mbsim/plot_elements/plot_rigid_body_group.h"
#include "mbsim/rigid_body.h"
#include "mbsim/frame.h"
#include "mbsim/environment.h"
#include "mbsim/utils/rotarymatrices.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/frame.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  PlotRigidBodyGroup::PlotRigidBodyGroup(const std::string &name) : Element(name), ref(0) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPosition=0;
    openMBVVelocity=0;
    openMBVAcceleration=0;
    openMBVAngularVelocity=0;
    openMBVAngularAcceleration=0;
    openMBVWeight=0;
    openMBVMomentum=0;
    openMBVAngularMomentum=0;
    openMBVDerivativeOfMomentum=0;
    openMBVDerivativeOfAngularMomentum=0;
#endif
  }

  void PlotRigidBodyGroup::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          openMBVGrp=new OpenMBV::Group();
          openMBVGrp->setName(name+"_Group");
          openMBVGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVGrp);
          if(openMBVPosition) {
            openMBVPosition->setName(name+"_Position");
            getOpenMBVGrp()->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelocity->setName(name+"_Velocity");
            getOpenMBVGrp()->addObject(openMBVVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAcceleration->setName(name+"_Acceleration");
            getOpenMBVGrp()->addObject(openMBVAcceleration);
          }
          if(openMBVAngularVelocity) {
            openMBVAngularVelocity->setName(name+"_AngularVelocity");
            getOpenMBVGrp()->addObject(openMBVAngularVelocity);
          }
          if(openMBVAngularAcceleration) {
            openMBVAngularAcceleration->setName(name+"_AngularAcceleration");
            getOpenMBVGrp()->addObject(openMBVAngularAcceleration);
          }
          if(openMBVWeight) {
            openMBVWeight->setName(name+"_Weight");
            getOpenMBVGrp()->addObject(openMBVWeight);
          }
          if(openMBVMomentum) {
            openMBVMomentum->setName(name+"_Momentum");
            getOpenMBVGrp()->addObject(openMBVMomentum);
          }
          if(openMBVAngularMomentum) {
            openMBVAngularMomentum->setName(name+"_AngularMomentum");
            getOpenMBVGrp()->addObject(openMBVAngularMomentum);
          }
          if(openMBVDerivativeOfMomentum) {
            openMBVDerivativeOfMomentum->setName(name+"_DerivativeOfMomentum");
            getOpenMBVGrp()->addObject(openMBVDerivativeOfMomentum);
          }
          if(openMBVDerivativeOfAngularMomentum) {
            openMBVDerivativeOfAngularMomentum->setName(name+"_DerivativeOfAngularMomentum");
            getOpenMBVGrp()->addObject(openMBVDerivativeOfAngularMomentum);
          }
        }
#endif
      }
      Element::init(stage);
    }
    else
      Element::init(stage);
  }

  void PlotRigidBodyGroup::enableOpenMBVWeight(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVWeight=new OpenMBV::Arrow;
      openMBVWeight->setScaleLength(scaleLength);
      openMBVWeight->setDiameter(diameter);
      openMBVWeight->setHeadDiameter(headDiameter);
      openMBVWeight->setHeadLength(headLength);
      openMBVWeight->setStaticColor(color);
    }
    else {
      openMBVWeight=0;
    }
  }

  void PlotRigidBodyGroup::enableOpenMBVMomentum(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVMomentum=new OpenMBV::Arrow;
      openMBVMomentum->setScaleLength(scaleLength);
      openMBVMomentum->setDiameter(diameter);
      openMBVMomentum->setHeadDiameter(headDiameter);
      openMBVMomentum->setHeadLength(headLength);
      openMBVMomentum->setStaticColor(color);
    }
    else {
      openMBVMomentum=0;
    }
  }

  void PlotRigidBodyGroup::enableOpenMBVAngularMomentum(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVAngularMomentum=new OpenMBV::Arrow;
      openMBVAngularMomentum->setScaleLength(scaleLength);
      openMBVAngularMomentum->setDiameter(diameter);
      openMBVAngularMomentum->setHeadDiameter(headDiameter);
      openMBVAngularMomentum->setHeadLength(headLength);
      openMBVAngularMomentum->setStaticColor(color);
    }
    else {
      openMBVAngularMomentum=0;
    }
  }

  void PlotRigidBodyGroup::enableOpenMBVDerivativeOfMomentum(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVDerivativeOfMomentum=new OpenMBV::Arrow;
      openMBVDerivativeOfMomentum->setScaleLength(scaleLength);
      openMBVDerivativeOfMomentum->setDiameter(diameter);
      openMBVDerivativeOfMomentum->setHeadDiameter(headDiameter);
      openMBVDerivativeOfMomentum->setHeadLength(headLength);
      openMBVDerivativeOfMomentum->setStaticColor(color);
    }
    else {
      openMBVDerivativeOfMomentum=0;
    }
  }

  void PlotRigidBodyGroup::enableOpenMBVDerivativeOfAngularMomentum(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVDerivativeOfAngularMomentum=new OpenMBV::Arrow;
      openMBVDerivativeOfAngularMomentum->setScaleLength(scaleLength);
      openMBVDerivativeOfAngularMomentum->setDiameter(diameter);
      openMBVDerivativeOfAngularMomentum->setHeadDiameter(headDiameter);
      openMBVDerivativeOfAngularMomentum->setHeadLength(headLength);
      openMBVDerivativeOfAngularMomentum->setStaticColor(color);
    }
    else {
      openMBVDerivativeOfAngularMomentum=0;
    }
  }

  void PlotRigidBodyGroup::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      double m = 0;
      for(unsigned int i=0; i<body.size(); i++) {
        m += body[i]->getMass();
      }
      Vec3 mpos(3), mvel(3), macc(3);
      Vec3 p = mvel;
      Vec3 pd = macc;
      Vec3 L(3), Ld(3);
      for(unsigned int i=0; i<body.size(); i++) {
        mpos += body[i]->getMass()*body[i]->getFrame("C")->getPosition();
        mvel += body[i]->getMass()*body[i]->getFrame("C")->getVelocity();
        macc += body[i]->getMass()*body[i]->getFrame("C")->getAcceleration();
      }
      Vec3 rOS = mpos/m;
      Vec3 vS = mvel/m;
      Vec3 aS = macc/m;
      Vec3 rOR = ref?ref->getPosition():rOS;
      Vec3 vR = ref?ref->getVelocity():vS;
      Vec3 aR = ref?ref->getAcceleration():aS;
      for(unsigned int i=0; i<body.size(); i++) {
        Mat3 AIK = body[i]->getFrame("C")->getOrientation();
        Vec3 rRSi = body[i]->getFrame("C")->getPosition() - rOR;
        Vec3 vSi = body[i]->getFrame("C")->getVelocity() - vR;
        Vec3 aSi = body[i]->getFrame("C")->getAcceleration() - aR;
        Vec3 omi = body[i]->getFrame("C")->getAngularVelocity();
        Vec3 psii = body[i]->getFrame("C")->getAngularAcceleration();
        double mi = body[i]->getMass();
        //L += (AIK*body[i]->getInertiaTensor()*AIK.T() - body[i]->getMass()*tilde(rRi)*tilde(rRi))*body[i]->getFrame("C")->getAngularVelocity();
        Mat3 WThetaS = AIK*body[i]->getInertiaTensor()*AIK.T();
        L += WThetaS*omi + crossProduct(rRSi,mi*vSi);
        Ld += WThetaS*psii + crossProduct(omi,WThetaS*omi) + crossProduct(rRSi,mi*aSi);
      }
      Vec G = m*MBSimEnvironment::getInstance()->getAccelerationOfGravity();
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVWeight) {
          vector<double> data;
          data.push_back(t);
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
          data.push_back(t);
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
          data.push_back(t);
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
          data.push_back(t);
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
          data.push_back(t);
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
      Element::plot(t,dt);
    }
  }
}
