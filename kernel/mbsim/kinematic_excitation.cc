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
#include "mbsim/kinematic_excitation.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  KinematicExcitation::KinematicExcitation(const string &name) : LinkMechanics(name), body(0)
  {
  }

  void KinematicExcitation::calclaSize(int j) {
    laSize = (*f)(0).size();
  }
  void KinematicExcitation::calcgSize(int j) {
    gSize = (*f)(0).size();
  }
  void KinematicExcitation::calcgdSize(int j) {
    gdSize = (*f)(0).size();
  }

  void KinematicExcitation::updateW(double t, int j) {
    if(j==0) {
      W[j][0]-=body->getJRel(j).T();
    } else {
      W[j][0]-=body->getFrameForKinematics()->getJacobianOfTranslation(j).T()*(body->getFrameOfReference()->getOrientation()*body->getPJT()) + body->getFrameForKinematics()->getJacobianOfRotation(j).T()*(body->getFrameOfReference()->getOrientation()*body->getPJR());
      W[j][1]+=C.getJacobianOfTranslation(j).T()*(body->getFrameOfReference()->getOrientation()*body->getPJT()) + C.getJacobianOfRotation(j).T()*(body->getFrameOfReference()->getOrientation()*body->getPJR());
    }
  }

  void KinematicExcitation::updateh(double t, int j) {
    la = (*func)(g,gd);
    if(j==0) {
      h[j][0]-=body->getJRel(j).T()*la;
    } else {
      h[j][0]-=body->getFrameForKinematics()->getJacobianOfTranslation(j).T()*(body->getFrameOfReference()->getOrientation()*body->getPJT()*la) + body->getFrameForKinematics()->getJacobianOfRotation(j).T()*(body->getFrameOfReference()->getOrientation()*body->getPJR()*la);
      h[j][1]+=C.getJacobianOfTranslation(j).T()*(body->getFrameOfReference()->getOrientation()*body->getPJT()*la) + C.getJacobianOfRotation(j).T()*(body->getFrameOfReference()->getOrientation()*body->getPJR()*la);
    }
  }

  void KinematicExcitation::updateWRef(const Mat &WParent, int j) {
    Index J = Index(laInd,laInd+laSize-1);
    Index I = Index(body->getFrameForKinematics()->gethInd(j),body->getFrameForKinematics()->gethInd(j)+body->getFrameForKinematics()->getJacobianOfTranslation(j).cols()-1);

    W[j][0]>>WParent(I,J);
    I = Index(body->getFrameOfReference()->gethInd(j),body->getFrameOfReference()->gethInd(j)+body->getFrameOfReference()->getJacobianOfTranslation(j).cols()-1);
    W[j][1]>>WParent(I,J);
  } 

  void KinematicExcitation::updatehRef(const Vec &hParent, int j) {
    Index I = Index(body->gethInd(j),body->gethInd(j)+body->gethSize(j)-1);
    h[j][0]>>hParent(I);
    I = Index(body->getFrameOfReference()->gethInd(j),body->getFrameOfReference()->gethInd(j)+body->getFrameOfReference()->getJacobianOfTranslation(j).cols()-1);
    h[j][1]>>hParent(I);
  } 

  void KinematicExcitation::updateJacobians(double t, int j) {
    Vec3 WrP0P1 = body->getFrameForKinematics()->getPosition()-body->getFrameOfReference()->getPosition();
    Mat33 tWrP0P1 = tilde(WrP0P1);

    C.setOrientation(body->getFrameOfReference()->getOrientation());
    C.setPosition(body->getFrameOfReference()->getPosition() + WrP0P1);
    C.setAngularVelocity(body->getFrameOfReference()->getAngularVelocity());
    C.setVelocity(body->getFrameOfReference()->getVelocity() + crossProduct(body->getFrameOfReference()->getAngularVelocity(),WrP0P1));
    C.setJacobianOfTranslation(body->getFrameOfReference()->getJacobianOfTranslation(j) - tWrP0P1*body->getFrameOfReference()->getJacobianOfRotation(j),j);
    C.setJacobianOfRotation(body->getFrameOfReference()->getJacobianOfRotation(j),j);
    C.setGyroscopicAccelerationOfTranslation(body->getFrameOfReference()->getGyroscopicAccelerationOfTranslation(j) - tWrP0P1*body->getFrameOfReference()->getGyroscopicAccelerationOfRotation(j) + crossProduct(body->getFrameOfReference()->getAngularVelocity(),crossProduct(body->getFrameOfReference()->getAngularVelocity(),WrP0P1)),j);
    C.setGyroscopicAccelerationOfRotation(body->getFrameOfReference()->getGyroscopicAccelerationOfRotation(j),j);
  }

  void KinematicExcitation::updateg(double t) {
    if(g.size())
    g=body->getqRel()-(*f)(t);
  } 

  void KinematicExcitation::updategd(double t) {
    if(gd.size())
    gd=body->getuRel()-(*fd)(t);
  }

  bool KinematicExcitation::isSetValued() const {
    return func?false:true;
  }

  void KinematicExcitation::updatewb(double t, int j) {
    wb += body->getjRel()-(*fdd)(t);
  }

 void KinematicExcitation::init(InitStage stage) {
    if(stage==unknownStage) {
      //LinkMechanics::init(stage);

      h[0].push_back(Vec(body->getFrameForKinematics()->getJacobianOfTranslation(0).cols()));
      h[1].push_back(Vec(6));
      W[0].push_back(Mat(body->getFrameForKinematics()->getJacobianOfTranslation(0).cols(),laSize));
      W[1].push_back(Mat(6,laSize));
      h[0].push_back(Vec(body->getFrameOfReference()->getJacobianOfTranslation(0).cols()));
      h[1].push_back(Vec(6));
      W[0].push_back(Mat(body->getFrameOfReference()->getJacobianOfTranslation(0).cols(),laSize));
      W[1].push_back(Mat(6,laSize));
      C.getJacobianOfTranslation(0).resize(body->getFrameOfReference()->getJacobianOfTranslation(0).cols());
      C.getJacobianOfRotation(0).resize(body->getFrameOfReference()->getJacobianOfRotation(0).cols());
      C.getJacobianOfTranslation(1).resize(body->getFrameOfReference()->getJacobianOfTranslation(1).cols());
      C.getJacobianOfRotation(1).resize(body->getFrameOfReference()->getJacobianOfRotation(1).cols());
    }
    else if(stage==resize) {
      LinkMechanics::init(stage);
      g.resize(gSize);
      gd.resize(gdSize);
      la.resize(laSize);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures();
      //plotColumns.push_back("la(0)");
      if(getPlotFeature(plotRecursive)==enabled) {
        LinkMechanics::init(stage);
      }
    }
    else {
      LinkMechanics::init(stage);
    }
  }

  void KinematicExcitation::plot(double t,double dt) {
    //plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
      LinkMechanics::plot(t,dt);
    }
  }

}


