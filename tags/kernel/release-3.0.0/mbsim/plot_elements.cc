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
#include "mbsim/plot_elements.h"
#include "mbsim/rigid_body.h"
#include "mbsim/frame.h"
#include "mbsim/environment.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/arrow.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  PlotFrame::PlotFrame(const std::string &name) : Element(name), frame(0) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVPosition=0;
    openMBVVelocity=0;
    openMBVAcceleration=0;
    openMBVAngularVelocity=0;
    openMBVAngularAcceleration=0;
#endif
  }

  void PlotFrame::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(openMBVPosition) {
            openMBVPosition->setName(name+"_Position");
            parent->getOpenMBVGrp()->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelocity->setName(name+"_Velocity");
            parent->getOpenMBVGrp()->addObject(openMBVVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAcceleration->setName(name+"_Acceleration");
            parent->getOpenMBVGrp()->addObject(openMBVAcceleration);
          }
          if(openMBVAngularVelocity) {
            openMBVAngularVelocity->setName(name+"_AngularVelocity");
            parent->getOpenMBVGrp()->addObject(openMBVAngularVelocity);
          }
          if(openMBVAngularAcceleration) {
            openMBVAngularAcceleration->setName(name+"_AngularAcceleration");
            parent->getOpenMBVGrp()->addObject(openMBVAngularAcceleration);
          }
        }
#endif
      }
      Element::init(stage);
    }
    else
      Element::init(stage);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void PlotFrame::enableOpenMBVPosition(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVPosition=new OpenMBV::Arrow;
      openMBVPosition->setScaleLength(scaleLength);
      openMBVPosition->setDiameter(diameter);
      openMBVPosition->setHeadDiameter(headDiameter);
      openMBVPosition->setHeadLength(headLength);
      openMBVPosition->setStaticColor(color);
      openMBVPosition->setType(OpenMBV::Arrow::fromHead);
    }
    else {
      openMBVPosition=0;
    }
  }

  void PlotFrame::enableOpenMBVVelocity(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVVelocity=new OpenMBV::Arrow;
      openMBVVelocity->setScaleLength(scaleLength);
      openMBVVelocity->setDiameter(diameter);
      openMBVVelocity->setHeadDiameter(headDiameter);
      openMBVVelocity->setHeadLength(headLength);
      openMBVVelocity->setStaticColor(color);
    }
    else {
      openMBVVelocity=0;
    }
  }

  void PlotFrame::enableOpenMBVAcceleration(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVAcceleration=new OpenMBV::Arrow;
      openMBVAcceleration->setScaleLength(scaleLength);
      openMBVAcceleration->setDiameter(diameter);
      openMBVAcceleration->setHeadDiameter(headDiameter);
      openMBVAcceleration->setHeadLength(headLength);
      openMBVAcceleration->setStaticColor(color);
    }
    else {
      openMBVAcceleration=0;
    }
  }

  void PlotFrame::enableOpenMBVAngularVelocity(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVAngularVelocity=new OpenMBV::Arrow;
      openMBVAngularVelocity->setScaleLength(scaleLength);
      openMBVAngularVelocity->setDiameter(diameter);
      openMBVAngularVelocity->setHeadDiameter(headDiameter);
      openMBVAngularVelocity->setHeadLength(headLength);
      openMBVAngularVelocity->setStaticColor(color);
    }
    else {
      openMBVAngularVelocity=0;
    }
  }

  void PlotFrame::enableOpenMBVAngularAcceleration(double scaleLength, double diameter, double headDiameter, double headLength, double color) {
    if(scaleLength>=0) {
      openMBVAngularAcceleration=new OpenMBV::Arrow;
      openMBVAngularAcceleration->setScaleLength(scaleLength);
      openMBVAngularAcceleration->setDiameter(diameter);
      openMBVAngularAcceleration->setHeadDiameter(headDiameter);
      openMBVAngularAcceleration->setHeadLength(headLength);
      openMBVAngularAcceleration->setStaticColor(color);
    }
    else {
      openMBVAngularAcceleration=0;
    }
  }
#endif

  void PlotFrame::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(openMBVPosition && !openMBVPosition->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(0);
          data.push_back(0);
          data.push_back(0);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(0.5);
          openMBVPosition->append(data);
        }
        if(openMBVVelocity && !openMBVVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getVelocity()(0));
          data.push_back(frame->getVelocity()(1));
          data.push_back(frame->getVelocity()(2));
          data.push_back(0.5);
          openMBVVelocity->append(data);
        }
        if(openMBVAcceleration && !openMBVAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAcceleration()(0));
          data.push_back(frame->getAcceleration()(1));
          data.push_back(frame->getAcceleration()(2));
          data.push_back(0.5);
          openMBVAcceleration->append(data);
        }
        if(openMBVAngularVelocity && !openMBVAngularVelocity->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAngularVelocity()(0));
          data.push_back(frame->getAngularVelocity()(1));
          data.push_back(frame->getAngularVelocity()(2));
          data.push_back(0.5);
          openMBVAngularVelocity->append(data);
        }
        if(openMBVAngularAcceleration && !openMBVAngularAcceleration->isHDF5Link()) {
          vector<double> data;
          data.push_back(t);
          data.push_back(frame->getPosition()(0));
          data.push_back(frame->getPosition()(1));
          data.push_back(frame->getPosition()(2));
          data.push_back(frame->getAngularAcceleration()(0));
          data.push_back(frame->getAngularAcceleration()(1));
          data.push_back(frame->getAngularAcceleration()(2));
          data.push_back(0.5);
          openMBVAngularAcceleration->append(data);
        }
      }
#endif

      Element::plot(t,dt);
    }
  }


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
          if(openMBVPosition) {
            openMBVPosition->setName(name+"_Position");
            parent->getOpenMBVGrp()->addObject(openMBVPosition);
          }
          if(openMBVVelocity) {
            openMBVVelocity->setName(name+"_Velocity");
            parent->getOpenMBVGrp()->addObject(openMBVVelocity);
          }
          if(openMBVAcceleration) {
            openMBVAcceleration->setName(name+"_Acceleration");
            parent->getOpenMBVGrp()->addObject(openMBVAcceleration);
          }
          if(openMBVAngularVelocity) {
            openMBVAngularVelocity->setName(name+"_AngularVelocity");
            parent->getOpenMBVGrp()->addObject(openMBVAngularVelocity);
          }
          if(openMBVAngularAcceleration) {
            openMBVAngularAcceleration->setName(name+"_AngularAcceleration");
            parent->getOpenMBVGrp()->addObject(openMBVAngularAcceleration);
          }
          if(openMBVWeight) {
            openMBVWeight->setName(name+"_Weight");
            parent->getOpenMBVGrp()->addObject(openMBVWeight);
          }
          if(openMBVMomentum) {
            openMBVMomentum->setName(name+"_Momentum");
            parent->getOpenMBVGrp()->addObject(openMBVMomentum);
          }
          if(openMBVAngularMomentum) {
            openMBVAngularMomentum->setName(name+"_AngularMomentum");
            parent->getOpenMBVGrp()->addObject(openMBVAngularMomentum);
          }
          if(openMBVDerivativeOfMomentum) {
            openMBVDerivativeOfMomentum->setName(name+"_DerivativeOfMomentum");
            parent->getOpenMBVGrp()->addObject(openMBVDerivativeOfMomentum);
          }
          if(openMBVDerivativeOfAngularMomentum) {
            openMBVDerivativeOfAngularMomentum->setName(name+"_DerivativeOfAngularMomentum");
            parent->getOpenMBVGrp()->addObject(openMBVDerivativeOfAngularMomentum);
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
      Vec mpos(3), mvel(3), macc(3);
      Vec p = mvel;
      Vec pd = macc;
      Vec L(3), Ld(3);
      for(unsigned int i=0; i<body.size(); i++) {
        mpos += body[i]->getMass()*body[i]->getFrame("C")->getPosition();
        mvel += body[i]->getMass()*body[i]->getFrame("C")->getVelocity();
        macc += body[i]->getMass()*body[i]->getFrame("C")->getAcceleration();
      }
      Vec rOS = mpos/m;
      Vec vS = mvel/m;
      Vec aS = macc/m;
      Vec rOR = ref?ref->getPosition():rOS;
      Vec vR = ref?ref->getVelocity():vS;
      Vec aR = ref?ref->getAcceleration():aS;
      for(unsigned int i=0; i<body.size(); i++) {
        SqrMat AIK = body[i]->getFrame("C")->getOrientation();
        Vec rRSi = body[i]->getFrame("C")->getPosition() - rOR;
        Vec vSi = body[i]->getFrame("C")->getVelocity() - vR;
        Vec aSi = body[i]->getFrame("C")->getAcceleration() - aR;
        Vec omi = body[i]->getFrame("C")->getAngularVelocity();
        Vec psii = body[i]->getFrame("C")->getAngularAcceleration();
        double mi = body[i]->getMass();
        //L += (AIK*body[i]->getInertiaTensor()*AIK.T() - body[i]->getMass()*tilde(rRi)*tilde(rRi))*body[i]->getFrame("C")->getAngularVelocity();
        Mat WThetaS = AIK*body[i]->getInertiaTensor()*AIK.T();
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