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
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  KinematicExcitation::KinematicExcitation(const string &name) : LinkMechanics(name), func(0), body(0) {

#ifdef HAVE_OPENMBVCPPINTERFACE
    FArrow = 0;
    MArrow = 0;
#endif
  }

  void KinematicExcitation::calclaSize(int j) {
    laSize = body->getuRelSize();
  }
  void KinematicExcitation::calcgSize(int j) {
    gSize = body->getuRelSize();
  }
  void KinematicExcitation::calcgdSize(int j) {
    gdSize = body->getuRelSize();
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
    Mat3x3 tWrP0P1 = tilde(WrP0P1);

    C.setOrientation(body->getFrameOfReference()->getOrientation());
    C.setPosition(body->getFrameOfReference()->getPosition() + WrP0P1);
    C.setAngularVelocity(body->getFrameOfReference()->getAngularVelocity());
    C.setVelocity(body->getFrameOfReference()->getVelocity() + crossProduct(body->getFrameOfReference()->getAngularVelocity(),WrP0P1));
    C.setJacobianOfTranslation(body->getFrameOfReference()->getJacobianOfTranslation(j) - tWrP0P1*body->getFrameOfReference()->getJacobianOfRotation(j),j);
    C.setJacobianOfRotation(body->getFrameOfReference()->getJacobianOfRotation(j),j);
    C.setGyroscopicAccelerationOfTranslation(body->getFrameOfReference()->getGyroscopicAccelerationOfTranslation(j) - tWrP0P1*body->getFrameOfReference()->getGyroscopicAccelerationOfRotation(j) + crossProduct(body->getFrameOfReference()->getAngularVelocity(),crossProduct(body->getFrameOfReference()->getAngularVelocity(),WrP0P1)),j);
    C.setGyroscopicAccelerationOfRotation(body->getFrameOfReference()->getGyroscopicAccelerationOfRotation(j),j);
  }

  bool KinematicExcitation::isSetValued() const {
    return func?false:true;
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
    else if(stage==plotting) {
      updatePlotFeatures();
      //plotColumns.push_back("la(0)");
      if(getPlotFeature(plotRecursive)==enabled) {
        LinkMechanics::init(stage);
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled) {
          if(FArrow) {
            FArrow->setName("Force");
            openMBVForceGrp->addObject(FArrow);
          }
          if(MArrow) {
            MArrow->setName("Moment");
            openMBVForceGrp->addObject(MArrow);
          }
        }
#endif
      }
    }
    else {
      LinkMechanics::init(stage);
    }
    if(func) func->init(stage);
  }

  void KinematicExcitation::plot(double t,double dt) {
    //plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled) {
        if(FArrow) {
          vector<double> data;
          data.push_back(t);
          Vec3 WF = -body->getFrameOfReference()->getOrientation()*body->getPJT()*la;
          Vec3 WrOS=body->getFrameC()->getPosition();
          data.push_back(WrOS(0));
          data.push_back(WrOS(1));
          data.push_back(WrOS(2));
          data.push_back(WF(0));
          data.push_back(WF(1));
          data.push_back(WF(2));
          data.push_back(1.0);
          FArrow->append(data);
        }
        if(MArrow) {
          vector<double> data;
          data.push_back(t);
          Vec3 WM = -body->getFrameOfReference()->getOrientation()*body->getPJR()*la;
          Vec3 WrOS=body->getFrameC()->getPosition();
          data.push_back(WrOS(0));
          data.push_back(WrOS(1));
          data.push_back(WrOS(2));
          data.push_back(WM(0));
          data.push_back(WM(1));
          data.push_back(WM(2));
          data.push_back(1.0);
          MArrow->append(data);
        }
      }
#endif
      LinkMechanics::plot(t,dt);
    }
  }

  void GeneralizedPositionExcitation::calcxSize() {
    if(!f) xSize = body->getqRelSize();
  }

  void GeneralizedPositionExcitation::updatexd(double t) {
    //if(!f && fd) xd = (*fd)(t);
  }

  void GeneralizedPositionExcitation::updateg(double t) {
    if(g.size()) g=body->getqRel()-(*f)(t);
  } 

  void GeneralizedPositionExcitation::updategd(double t) {
    if(gd.size()) gd=body->getuRel()-f->parDer(t);
  }

  void GeneralizedPositionExcitation::updatewb(double t, int j) {
    wb += body->getjRel()-f->parDerParDer(t);
  }

  void GeneralizedVelocityExcitation::calcxSize() {
    xSize = body->getqRelSize();
  }

  void GeneralizedVelocityExcitation::updatexd(double t) {
    if(f) xd = (*f)(x,t);
  }

  void GeneralizedVelocityExcitation::updateg(double t) {
    if(g.size()) g=body->getqRel()-x;
  } 

  void GeneralizedVelocityExcitation::updategd(double t) {
    if(gd.size()) gd=body->getuRel()-(*f)(x,t);
  }

  void GeneralizedVelocityExcitation::updatewb(double t, int j) {
    wb += body->getjRel()-(f->parDer1(x,t)*xd + f->parDer2(x,t));
  }

  void GeneralizedAccelerationExcitation::calcxSize() {
    xSize = body->getqRelSize()+body->getuRelSize();
  }

  void GeneralizedAccelerationExcitation::updatexd(double t) {
    xd(0,body->getqRelSize()-1) = x(body->getqRelSize(),body->getqRelSize()+body->getuRelSize()-1);
    xd(body->getqRelSize(),body->getqRelSize()+body->getuRelSize()-1) = (*f)(x,t);
  }

  void GeneralizedAccelerationExcitation::updateg(double t) {
    if(g.size()) g=body->getqRel()-x(0,body->getqRelSize()-1);
  } 

  void GeneralizedAccelerationExcitation::updategd(double t) {
    if(gd.size()) gd=body->getuRel()-x(body->getqRelSize(),body->getqRelSize()+body->getuRelSize()-1);
  }

  void GeneralizedAccelerationExcitation::updatewb(double t, int j) {
    wb += (*f)(x,t);
  }


}


