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
#include "mbsim/gear.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Gear::Gear(const string &name) : LinkMechanics(name)
  {
    body.push_back(0); 
    ratio.push_back(-1);
  }

  void Gear::calclaSize(int j) {
    laSize = 1;
  }
  void Gear::calcgSize(int j) {
    gSize = 1;
  }
  void Gear::calcgdSize(int j) {
    gdSize = 1;
  }

  void Gear::addDependency(RigidBody* body_, double ratio_) {
    body.push_back(body_); 
    ratio.push_back(ratio_);
  }

  void Gear::updateW(double t, int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)  {
        W[j][i]-=body[i]->getJRel(j).T()*ratio[i];
      }
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        W[j][i]-=body[i]->getFrameForKinematics()->getJacobianOfTranslation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJT()*ratio[i]) + body[i]->getFrameForKinematics()->getJacobianOfRotation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[i]);
        W[j][body.size()+i]+=C[i].getJacobianOfTranslation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJT()*ratio[i]) + C[i].getJacobianOfRotation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[i]);
      }
    }
  }

  void Gear::updateh(double t, int j) {
    la(0) = (*func)(g(0),gd(0));
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)  {
        h[j][i]-=body[i]->getJRel(j).T()*ratio[i]*la;
      }
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        h[j][i]-=body[i]->getFrameForKinematics()->getJacobianOfTranslation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJT()*ratio[i]*la) + body[i]->getFrameForKinematics()->getJacobianOfRotation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[i]*la);
        h[j][body.size()+i]+=C[i].getJacobianOfTranslation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJT()*ratio[i]*la) + C[i].getJacobianOfRotation(j).T()*(body[i]->getFrameOfReference()->getOrientation()*body[i]->getPJR()*ratio[i]*la);
      }
    }
  }

  void Gear::updateWRef(const Mat &WParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(body[i]->getFrameForKinematics()->gethInd(j),body[i]->getFrameForKinematics()->gethInd(j)+body[i]->getFrameForKinematics()->getJacobianOfTranslation(j).cols()-1);

      W[j][i]>>WParent(I,J);
      I = Index(body[i]->getFrameOfReference()->gethInd(j),body[i]->getFrameOfReference()->gethInd(j)+body[i]->getFrameOfReference()->getJacobianOfTranslation(j).cols()-1);
      W[j][body.size()+i]>>WParent(I,J);
    }
  } 

  void Gear::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Index I = Index(body[i]->gethInd(j),body[i]->gethInd(j)+body[i]->gethSize(j)-1);
      h[j][i]>>hParent(I);
      I = Index(body[i]->getFrameOfReference()->gethInd(j),body[i]->getFrameOfReference()->gethInd(j)+body[i]->getFrameOfReference()->getJacobianOfTranslation(j).cols()-1);
      h[j][body.size()+i]>>hParent(I);
    }
  } 

  void Gear::updateJacobians(double t, int j) {
    for(unsigned i=0; i<body.size(); i++) {
      Vec3 WrP0P1 = body[i]->getFrameForKinematics()->getPosition()-body[i]->getFrameOfReference()->getPosition();
      Mat33 tWrP0P1 = tilde(WrP0P1);

      C[i].setOrientation(body[i]->getFrameOfReference()->getOrientation());
      C[i].setPosition(body[i]->getFrameOfReference()->getPosition() + WrP0P1);
      C[i].setAngularVelocity(body[i]->getFrameOfReference()->getAngularVelocity());
      C[i].setVelocity(body[i]->getFrameOfReference()->getVelocity() + crossProduct(body[i]->getFrameOfReference()->getAngularVelocity(),WrP0P1));
      C[i].setJacobianOfTranslation(body[i]->getFrameOfReference()->getJacobianOfTranslation(j) - tWrP0P1*body[i]->getFrameOfReference()->getJacobianOfRotation(j),j);
      C[i].setJacobianOfRotation(body[i]->getFrameOfReference()->getJacobianOfRotation(j),j);
      C[i].setGyroscopicAccelerationOfTranslation(body[i]->getFrameOfReference()->getGyroscopicAccelerationOfTranslation(j) - tWrP0P1*body[i]->getFrameOfReference()->getGyroscopicAccelerationOfRotation(j) + crossProduct(body[i]->getFrameOfReference()->getAngularVelocity(),crossProduct(body[i]->getFrameOfReference()->getAngularVelocity(),WrP0P1)),j);
      C[i].setGyroscopicAccelerationOfRotation(body[i]->getFrameOfReference()->getGyroscopicAccelerationOfRotation(j),j);
    }
  }

  void Gear::updateg(double) {
    g.init(0);
    for(unsigned i=0; i<body.size(); i++)
      g+=body[i]->getqRel()*ratio[i];
  } 

  void Gear::updategd(double) {
    gd.init(0);
    for(unsigned i=0; i<body.size(); i++) {
      gd+=body[i]->getuRel()*ratio[i];
    }
  }

  bool Gear::isSetValued() const {
    return func?false:true;
  }

  void Gear::updatewb(double t, int j) {
    for(unsigned i=0; i<body.size(); i++)
      wb += body[i]->getjRel()*ratio[i];
  }

 void Gear::init(InitStage stage) {
    if(stage==unknownStage) {
      //LinkMechanics::init(stage);

      for(unsigned int i=0; i<body.size(); i++) {
        h[0].push_back(Vec(body[i]->getFrameForKinematics()->getJacobianOfTranslation(0).cols()));
        h[1].push_back(Vec(6));
        W[0].push_back(Mat(body[i]->getFrameForKinematics()->getJacobianOfTranslation(0).cols(),laSize));
        W[1].push_back(Mat(6,laSize));
        C.push_back(Frame());
        C[i].getJacobianOfTranslation(0).resize(body[i]->getFrameOfReference()->getJacobianOfTranslation(0).cols());
        C[i].getJacobianOfRotation(0).resize(body[i]->getFrameOfReference()->getJacobianOfRotation(0).cols());
        C[i].getJacobianOfTranslation(1).resize(body[i]->getFrameOfReference()->getJacobianOfTranslation(1).cols());
        C[i].getJacobianOfRotation(1).resize(body[i]->getFrameOfReference()->getJacobianOfRotation(1).cols());
      }
      for(unsigned int i=0; i<body.size(); i++) {
        h[0].push_back(Vec(body[i]->getFrameOfReference()->getJacobianOfTranslation(0).cols()));
        h[1].push_back(Vec(6));
        W[0].push_back(Mat(body[i]->getFrameOfReference()->getJacobianOfTranslation(0).cols(),laSize));
        W[1].push_back(Mat(6,laSize));
      }
    }
    else if(stage==resize) {
      LinkMechanics::init(stage);
      g.resize(1);
      gd.resize(1);
      la.resize(1);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
      plotColumns.push_back("la(0)");
      for(unsigned int i=0; i<body.size(); i++) {
        plotColumns.push_back("M");
      }
      if(getPlotFeature(plotRecursive)==enabled) {
        LinkMechanics::init(stage);
      }
    }
    else {
      LinkMechanics::init(stage);
    }
  }

  void Gear::plot(double t,double dt) {
    plotVector.push_back(la(0));
    for(unsigned int i=0; i<body.size(); i++) {
      plotVector.push_back(ratio[i]*la(0));
    }
    if(getPlotFeature(plotRecursive)==enabled) {
      LinkMechanics::plot(t,dt);
    }
  }

}


