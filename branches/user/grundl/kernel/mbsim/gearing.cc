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
#include "mbsim/gearing.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Gearing::Gearing(const string &name, bool flag_) : LinkMechanics(name), r0(1), r1(1), flag(flag_), Z0("Z0"), Z1("Z1") {
    Z0.enableOpenMBV();
    Z1.enableOpenMBV();
  }

  //void Gearing::connect(Frame* Z_, Frame* P0_, Frame* P1_) {
  //  Z = Z_;
  //  P0 = P0_;
  //  P1 = P1_;
  //}
  void Gearing::connect(double r0_, Frame* P0_, double r1_, Frame* P1_) {
    r0 = r0_;
    r1 = r1_;
    P0 = P0_;
    P1 = P1_;
  }
  void Gearing::calclaSize(int j) {
    laSize = 1;
  }
  void Gearing::calcgSize(int j) {
    gSize = 1;
  }
  void Gearing::calcgdSize(int j) {
    gdSize = 1;
  }

  void Gearing::updateJacobians(double t, int j) {
    Mat3x3 tWrP0Z = tilde(WrP0Z);
    Mat3x3 tWrP1Z = tilde(WrP1Z);

    Z0.setJacobianOfTranslation(P0->getJacobianOfTranslation(j) - tWrP0Z*P0->getJacobianOfRotation(j),j);
    Z0.setJacobianOfRotation(P0->getJacobianOfRotation(j),j);
    Z0.setGyroscopicAccelerationOfTranslation(P0->getGyroscopicAccelerationOfTranslation(j) - tWrP0Z*P0->getGyroscopicAccelerationOfRotation(j) + crossProduct(P0->getAngularVelocity(),crossProduct(P0->getAngularVelocity(),WrP0Z)),j);
    Z0.setGyroscopicAccelerationOfRotation(P0->getGyroscopicAccelerationOfRotation(j),j);

    Z1.setJacobianOfTranslation(P1->getJacobianOfTranslation(j) - tWrP1Z*P1->getJacobianOfRotation(j),j);
    Z1.setJacobianOfRotation(P1->getJacobianOfRotation(j),j);
    Z1.setGyroscopicAccelerationOfTranslation(P1->getGyroscopicAccelerationOfTranslation(j) - tWrP1Z*P1->getGyroscopicAccelerationOfRotation(j) + crossProduct(P1->getAngularVelocity(),crossProduct(P1->getAngularVelocity(),WrP1Z)),j);
    Z1.setGyroscopicAccelerationOfRotation(P1->getGyroscopicAccelerationOfRotation(j),j);
  }

  void Gearing::updateW(double t, int j) {
    W[j][0] += Z0.getJacobianOfTranslation(j).T()*Wt;
    W[j][1] -= Z1.getJacobianOfTranslation(j).T()*Wt;
  }

  void Gearing::updateh(double t, int j) {
    la(0) = (*func)(g(0),gd(0));
    h[j][0] += Z0.getJacobianOfTranslation(j).T()*Wt*la(0);
    h[j][1] -= Z1.getJacobianOfTranslation(j).T()*Wt*la(0);
  }

  void Gearing::updateWRef(const Mat &WParent, int j) {
    Index J = Index(laInd,laInd+laSize-1);
    Index I0 = Index(P0->gethInd(j),P0->gethInd(j)+P0->getJacobianOfTranslation(j).cols()-1); // TODO Prüfen ob hSize

    W[j][0]>>WParent(I0,J);

    Index I1 = Index(P1->gethInd(j),P1->gethInd(j)+P1->getJacobianOfTranslation(j).cols()-1); // TODO Prüfen ob hSize

    W[j][1]>>WParent(I1,J);
  } 

  void Gearing::updatehRef(const Vec &hParent, int j) {
    Index I0 = Index(P0->gethInd(j),P0->gethInd(j)+P0->getJacobianOfTranslation(j).cols()-1); // TODO Prüfen ob hSize

    h[j][0]>>hParent(I0);

    Index I1 = Index(P1->gethInd(j),P1->gethInd(j)+P1->getJacobianOfTranslation(j).cols()-1); // TODO Prüfen ob hSize

    h[j][1]>>hParent(I1);
  } 

  void Gearing::updateg(double) {
    //WrP0Z = Z->getPosition()-P0->getPosition();
    //WrP1Z = Z->getPosition()-P1->getPosition();
    Vec3 WrP0P1 = P1->getPosition()-P0->getPosition();
    Vec3 dir =  WrP0P1/nrm2(WrP0P1);
    WrP0Z = (flag?-1.:1.)*dir*r0;
    WrP1Z = -1.*dir*r1;
    //WrP0Z = WrP0P1/(1.+ratio);
    //WrP1Z = -WrP0P1*(ratio/(1.+ratio)); 
    //WrP1Z = WrP0Z - WrP0P1;
    Z0.setOrientation(P0->getOrientation());
    Z0.setPosition(P0->getPosition() + WrP0Z);
    Z1.setOrientation(P1->getOrientation());
    Z1.setPosition(P1->getPosition() + WrP1Z);
    g = x;
  } 

  void Gearing::updategd(double) {
    Z0.setAngularVelocity(P0->getAngularVelocity());
    Z0.setVelocity(P0->getVelocity() + crossProduct(P0->getAngularVelocity(),WrP0Z));
    Z1.setAngularVelocity(P1->getAngularVelocity());
    Z1.setVelocity(P1->getVelocity() + crossProduct(P1->getAngularVelocity(),WrP1Z));

    Vec3 WvZ0Z1 = Z1.getVelocity()-Z0.getVelocity();
    RigidBody* bodyP0 = dynamic_cast<RigidBody*>(P0->getParent());
    Mat3xV a = bodyP0->getFrameOfReference()->getOrientation()*bodyP0->getPJR();
    Wt = crossProduct(WrP0Z,a.col(0));
    Wt /= -nrm2(Wt);
    gd(0)=Wt.T()*WvZ0Z1;
  }

  bool Gearing::isSetValued() const {
    return func?false:true;
  }

  void Gearing::updatewb(double t, int j) {
    const Vec3 KrPC1 = P0->getOrientation().T()*(Z0.getPosition() - P0->getPosition());
    const double zeta1=(KrPC1(1)>0) ? acos(KrPC1(0)/nrm2(KrPC1)) : 2.*M_PI - acos(KrPC1(0)/nrm2(KrPC1));
    const double sa1=sin(zeta1);
    const double ca1=cos(zeta1);
    const double r1=nrm2(WrP0Z);
    Vec3 Ks1(NONINIT);
    Ks1(0)=-r1*sa1;
    Ks1(1)=r1*ca1;
    Ks1(2)=0;
    Vec3 Kt1(NONINIT);
    Kt1(0)=0;
    Kt1(1)=0;
    Kt1(2)=1;
    const Vec3 s1=P0->getOrientation()*Ks1;
    const Vec3 t1=P0->getOrientation()*Kt1;
    Vec3 n1=crossProduct(s1, t1);
    n1/=nrm2(n1);
    const Vec3 u1=s1/nrm2(s1);
    const Vec3 &R1 = s1;
    Vec KN1(3,NONINIT);
    KN1(0)=-sa1;
    KN1(1)=ca1;
    KN1(2)=0;
    const Vec3 N1=P0->getOrientation()*KN1;
    Vec KU1(3,NONINIT);
    KU1(0)=-ca1;
    KU1(1)=-sa1;
    KU1(2)=0;
    const Vec3 U1=P0->getOrientation()*KU1;

    const Vec3 KrPC2 = P1->getOrientation().T()*(Z1.getPosition() - P1->getPosition());
    const double zeta2=(KrPC2(1)>0) ? acos(KrPC2(0)/nrm2(KrPC2)) : 2.*M_PI - acos(KrPC2(0)/nrm2(KrPC2));
    const double sa2=sin(zeta2);
    const double ca2=cos(zeta2);
    const double r2=nrm2(WrP1Z);
    Vec3 Ks2(NONINIT);
    Ks2(0)=-r2*sa2;
    Ks2(1)=r2*ca2;
    Ks2(2)=0;
    Vec3 Kt2(NONINIT);
    Kt2(0)=0;
    Kt2(1)=0;
    Kt2(2)=1;
    const Vec3 s2=P1->getOrientation()*Ks2;
    const Vec3 t2=P1->getOrientation()*Kt2;
    Vec3 n2=(flag?-1.:1.)*crossProduct(s2, t2);
    n2/=nrm2(n2);
    const Vec3 u2=s2/nrm2(s2);
    const Vec3 v2=crossProduct(n2, u2);
    const Vec3 &R2 = s2;
    Vec3 KU2(NONINIT);
    KU2(0)=-ca2;
    KU2(1)=-sa2;
    KU2(2)=0;
    const Vec3 U2=P1->getOrientation()*KU2;

    const Vec3 vC1 = Z0.getVelocity();
    const Vec3 vC2 = Z1.getVelocity();
    const Vec3 Om1 = Z0.getAngularVelocity();
    const Vec3 Om2 = Z1.getAngularVelocity();

    SqrMat A(2,NONINIT);
    A(0,0)=-u1.T()*R1;
    A(0,1)=u1.T()*R2;
    A(1,0)=u2.T()*N1;
    A(1,1)=n1.T()*U2;
    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-v2.T()*(Om2-Om1);
    const Vec zetad = slvLU(A,b);

    const Mat3x3 tOm1 = tilde(Om1);
    const Mat3x3 tOm2 = tilde(Om2);
    
    wb(0) += Wt.T()*(Z1.getGyroscopicAccelerationOfTranslation(j) - Z0.getGyroscopicAccelerationOfTranslation(j)); 
    wb(0) += ((vC2-vC1).T()*U1-u1.T()*tOm1*R1)*zetad(0)+u1.T()*tOm2*R2*zetad(1)-u1.T()*tOm1*(vC2-vC1);
  }

  void Gearing::init(InitStage stage) {
    if(stage==unknownStage) {
      LinkMechanics::init(stage);

      h[0].push_back(Vec(P0->getJacobianOfTranslation().cols()));
      h[1].push_back(Vec(6));
      h[0].push_back(Vec(P1->getJacobianOfTranslation().cols()));
      h[1].push_back(Vec(6));

      W[0].push_back(Mat(P0->getJacobianOfTranslation().cols(),laSize));
      W[1].push_back(Mat(6,laSize));
      W[0].push_back(Mat(P1->getJacobianOfTranslation().cols(),laSize));
      W[1].push_back(Mat(6,laSize));
      Z0.getJacobianOfTranslation(0).resize(P0->getJacobianOfTranslation(0).cols());
      Z0.getJacobianOfTranslation(1).resize(P0->getJacobianOfTranslation(1).cols());
      Z1.getJacobianOfTranslation(0).resize(P1->getJacobianOfTranslation(0).cols());
      Z1.getJacobianOfTranslation(1).resize(P1->getJacobianOfTranslation(1).cols());
      Z0.getJacobianOfRotation(0).resize(P0->getJacobianOfRotation(0).cols());
      Z0.getJacobianOfRotation(1).resize(P0->getJacobianOfRotation(1).cols());
      Z1.getJacobianOfRotation(0).resize(P1->getJacobianOfRotation(0).cols());
      Z1.getJacobianOfRotation(1).resize(P1->getJacobianOfRotation(1).cols());
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
      if(getPlotFeature(plotRecursive)==enabled) {
        LinkMechanics::init(stage);
      }
    }
    else {
      LinkMechanics::init(stage);
    }
  }

  void Gearing::plot(double t,double dt) {
    plotVector.push_back(la(0));
    if(getPlotFeature(plotRecursive)==enabled) {
      LinkMechanics::plot(t,dt);
    }
  }

  void Gearing::updatexd(double t) {
    xd = gd;
  }

  void Gearing::updatedx(double t, double dt) {
    xd = gd*dt;
  }

  void Gearing::calcxSize() {
    LinkMechanics::calcxSize();
    xSize = 1;
  }
}


