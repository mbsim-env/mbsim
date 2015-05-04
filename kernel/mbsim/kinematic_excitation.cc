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
#include "mbsim/fixed_relative_frame.h"
#include "mbsim/dynamic_system_solver.h"
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  KinematicExcitation::KinematicExcitation(const string &name) : MechanicalLink(name), func(0), body(0), C("F") {
    C.setParent(this);
    C.setUpdateGlobalRelativePositionByParent();
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

  void KinematicExcitation::updatePositions(double t) {
    WrP0P1 = body->getFrameForKinematics()->getPosition(t)-body->getFrameOfReference()->getPosition(t);
    C.setGlobalRelativePosition(WrP0P1);
  }

  void KinematicExcitation::updateg(double) {
    g = rrel;
  }

  void KinematicExcitation::updategd(double) {
    gd = vrel;
  }

  void KinematicExcitation::updateW(double t, int j) {
    if(j==0) {
      W[j][0]-=body->getJRel(t,j).T();
    } else {
      W[j][0]-=body->getFrameForKinematics()->getJacobianOfTranslation(t,j).T()*(body->getFrameOfReference()->getOrientation(t)*body->getPJT(t)) + body->getFrameForKinematics()->getJacobianOfRotation(t,j).T()*(body->getFrameOfReference()->getOrientation(t)*body->getPJR(t));
      W[j][1]+=C.getJacobianOfTranslation(t,j).T()*(body->getFrameOfReference()->getOrientation(t)*body->getPJT(t)) + C.getJacobianOfRotation(t,j).T()*(body->getFrameOfReference()->getOrientation(t)*body->getPJR(t));
    }
  }

  void KinematicExcitation::updateGeneralizedSetValuedForces(double t) {
    laMV = la;
    updlaMV = false;
  }

  void KinematicExcitation::updateGeneralizedSingleValuedForces(double t) {
    laSV = (*func)(getRelativePosition(t),getRelativeVelocity(t));
    updlaSV = false;
  }

  void KinematicExcitation::updateh(double t, int j) {
    if(j==0) {
      h[j][0]-=body->getJRel(t,j).T()*getSingleValuedForce(t);
    } else {
      Vec3 WF = body->getFrameOfReference()->getOrientation(t)*body->getPJT(t)*getSingleValuedGeneralizedForce(t);
      Vec3 WM = body->getFrameOfReference()->getOrientation(t)*body->getPJR(t)*getSingleValuedGeneralizedForce(t);
      h[j][0]-=body->getFrameForKinematics()->getJacobianOfTranslation(t,j).T()*WF + body->getFrameForKinematics()->getJacobianOfRotation(t,j).T()*WM;
      h[j][1]+=C.getJacobianOfTranslation(t,j).T()*WF + C.getJacobianOfRotation(t,j).T()*WM;
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

  bool KinematicExcitation::isSetValued() const {
    return func?false:true;
  }

 void KinematicExcitation::init(InitStage stage) {
    if(stage==unknownStage) {
      //MechanicalLink::init(stage);

      h[0].push_back(Vec(body->getFrameForKinematics()->getJacobianOfTranslation(0).cols()));
      h[1].push_back(Vec(6));
      W[0].push_back(Mat(body->getFrameForKinematics()->getJacobianOfTranslation(0).cols(),laSize));
      W[1].push_back(Mat(6,laSize));
      h[0].push_back(Vec(body->getFrameOfReference()->getJacobianOfTranslation(0).cols()));
      h[1].push_back(Vec(6));
      W[0].push_back(Mat(body->getFrameOfReference()->getJacobianOfTranslation(0).cols(),laSize));
      W[1].push_back(Mat(6,laSize));
      C.setParent(this);
      C.setUpdateGlobalRelativePositionByParent();
    }
    else if(stage==resize) {
      MechanicalLink::init(stage);
      rrel.resize(gSize);
      vrel.resize(gdSize);
      if(isSetValued()) {
        g.resize(gSize);
        gd.resize(gdSize);
        la.resize(laSize);
        laMV.resize(laSize);
      }
      else
        laSV.resize(laSize);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      //plotColumns.push_back("la(0)");
      if(getPlotFeature(plotRecursive)==enabled) {
        MechanicalLink::init(stage);
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
      MechanicalLink::init(stage);
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
      MechanicalLink::plot(t,dt);
    }
  }

  void GeneralizedPositionExcitation::calcxSize() {
    if(!f) xSize = body->getqRelSize();
  }

  void GeneralizedPositionExcitation::init(InitStage stage) {
    if(stage==preInit) {
      KinematicExcitation::init(stage);
      addDependency(f->getDependency());
    }
    else
      KinematicExcitation::init(stage);
    f->init(stage);
  }

  void GeneralizedPositionExcitation::updatePositions(double t) {
    KinematicExcitation::updatePositions(t);
    rrel=body->getqRel(t)-(*f)(t);
    updPos = false;
  } 

  void GeneralizedPositionExcitation::updateVelocities(double t) {
    vrel=body->getuRel(t)-f->parDer(t);
    updVel = false;
  }

  void GeneralizedPositionExcitation::updatewb(double t) {
    wb += body->getjRel(t)-f->parDerParDer(t);
  }

  void GeneralizedVelocityExcitation::calcxSize() {
    xSize = body->getqRelSize();
  }

  void GeneralizedVelocityExcitation::init(InitStage stage) {
    if(stage==preInit) {
      KinematicExcitation::init(stage);
      addDependency(f->getDependency());
    }
    else
      KinematicExcitation::init(stage);
    f->init(stage);
  }

  void GeneralizedVelocityExcitation::updatexd(double t) {
    if(f) xd = (*f)(x,t);
  }

  void GeneralizedVelocityExcitation::updatePositions(double t) {
    KinematicExcitation::updatePositions(t);
    rrel=body->getqRel(t)-x;
    updPos = false;
  } 

  void GeneralizedVelocityExcitation::updateVelocities(double t) {
    vrel=body->getuRel(t)-(*f)(x,t);
    updVel = false;
  }

  void GeneralizedVelocityExcitation::updatewb(double t) {
    wb += body->getjRel(t)-(f->parDer1(x,t)*xd + f->parDer2(x,t));
  }

  void GeneralizedAccelerationExcitation::calcxSize() {
    xSize = body->getqRelSize()+body->getuRelSize();
  }

  void GeneralizedAccelerationExcitation::init(InitStage stage) {
    if(stage==preInit) {
      KinematicExcitation::init(stage);
      addDependency(f->getDependency());
    }
    else
      KinematicExcitation::init(stage);
    f->init(stage);
  }

  void GeneralizedAccelerationExcitation::updatexd(double t) {
    xd(0,body->getqRelSize()-1) = x(body->getqRelSize(),body->getqRelSize()+body->getuRelSize()-1);
    xd(body->getqRelSize(),body->getqRelSize()+body->getuRelSize()-1) = (*f)(x,t);
  }

  void GeneralizedAccelerationExcitation::updatePositions(double t) {
    KinematicExcitation::updatePositions(t);
    rrel=body->getqRel(t)-x(0,body->getqRelSize()-1);
    updPos = false;
  }

  void GeneralizedAccelerationExcitation::updateVelocities(double t) {
    vrel=body->getuRel(t)-x(body->getqRelSize(),body->getqRelSize()+body->getuRelSize()-1);
    updVel = false;
  }

  void GeneralizedAccelerationExcitation::updatewb(double t) {
    wb += (*f)(x,t);
  }

}


