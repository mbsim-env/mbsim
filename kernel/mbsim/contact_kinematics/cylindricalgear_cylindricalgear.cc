/* Copyright (C) 2004-2018 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "cylindricalgear_cylindricalgear.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/cylindrical_gear.h"
#include <mbsim/utils/rotarymatrices.h>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCylindricalGearCylindricalGear::assignContours(const vector<Contour*> &contour) {
    if(not(static_cast<CylindricalGear*>(contour[0])->getExternalToothed())) {
      gear[0] = static_cast<CylindricalGear*>(contour[1]);
      gear[1] = static_cast<CylindricalGear*>(contour[0]);
      igear[0] = 1;
      igear[1] = 0;
    }
    else {
      gear[0] = static_cast<CylindricalGear*>(contour[0]);
      gear[1] = static_cast<CylindricalGear*>(contour[1]);
      igear[0] = 0;
      igear[1] = 1;
    }
    m = gear[0]->getModule()/cos(gear[0]->getHelixAngle());
    al0 = atan(tan(gear[0]->getPressureAngle())/cos(gear[0]->getHelixAngle()));
    double phi0 = tan(al0)-al0;
    double s0 = m*M_PI/2;
    double dk[2];
    for(int i=0; i<2; i++) {
      z[i] = gear[i]->getNumberOfTeeth();
      d0[i] = m*z[i];
      db[i] = d0[i]*cos(al0);
      dk[i] = d0[i]+2*m;
      rb[i] = db[i]/2;
      sb[i] = db[i]*(s0/d0[i]+phi0)-((i==1 and not gear[1]->getExternalToothed())?-gear[i]->getBacklash():gear[i]->getBacklash());
      ga[i] = sb[i]/rb[i]/2;
      beta[i] = gear[i]->getHelixAngle();
      delmin[i] = -al0-ga[i];
      delmax[i] = tan(acos(db[i]/dk[i]))-al0-ga[i];
    }
  }

  void ContactKinematicsCylindricalGearCylindricalGear::updateg(SingleContact &contact, int ii) {
    contact.getGeneralizedRelativePosition(false)(0) = 1e10;
    double eta[2], del[2];
    Vec3 rS1S2 = gear[1]->getExternalToothed()?(gear[1]->getFrame()->evalPosition() - gear[0]->getFrame()->evalPosition()):(gear[0]->getFrame()->evalPosition() - gear[1]->getFrame()->evalPosition());
    double a = nrm2(rS1S2);
    double a0 = ((gear[1]->getExternalToothed()?d0[0]:-d0[0])+d0[1])/2;
    al = acos(a0/a*cos(al0));

    for(int i=0; i<2; i++) {
      int signi = i?-1:1;
      Vec3 rOP[2], rSP[2];
      vector<int> v[2];
      if(maxNumContacts==1) {
        Vec3 rSPsave;
        for(int j=0; j<2; j++) {
          Vec3 rP1S2 = rS1S2 - (gear[1]->getExternalToothed()?rSPsave:-rSPsave);
          int signj = j?(gear[1]->getExternalToothed()?-1:1):1;
          double cdel = 0;
          int k = 0;
          for(int k_=0; k_<z[j]; k_++) {
            double ep = k_*2*M_PI/z[j]+signi*ga[j];
            rSP[j](0) = -sin(ep);
            rSP[j](1) = cos(ep);
            rSP[j] = gear[j]->getFrame()->getOrientation()*rSP[j];
            double cdel_ = signj*(rSP[j].T()*rP1S2/a);
            if(cdel_>cdel) {
              cdel = cdel_;
              k = k_;
              rSPsave = rb[0]*rSP[0];
            }
          }
          v[j].push_back(k);
        }
      }
      else {
        for(int j=0; j<2; j++) {
          int signj = j?(gear[1]->getExternalToothed()?-1:1):1;
          for(int k_=0; k_<z[j]; k_++) {
            double ep = k_*2*M_PI/z[j];
            rSP[j](0) = -sin(ep);
            rSP[j](1) = cos(ep);
            rSP[j] = gear[j]->getFrame()->getOrientation()*rSP[j];
            double cdel = signj*(rSP[j].T()*rS1S2/a);
            double del = signi*signj*(gear[j]->getFrame()->getOrientation().col(2).T()*crossProduct(rS1S2,rSP[j])>=0?acos(cdel):-acos(cdel));
            if(del>delmin[j] and del<delmax[j])
              v[j].push_back(k_);
          }
        }
      }

      double k[2];
      for (auto & i0 : v[0]) {
        for (auto & i1 : v[1]) {
          k[0] = i0;
          k[1] = i1;
          if(ii==0 or not(k[0]==ksave[0][0] and k[1]==ksave[0][1])) {
            for(int j=0; j<2; j++) {
              int signj = j?(gear[1]->getExternalToothed()?-1:1):1;
              rSP[j](0) = -sin(k[j]*2*M_PI/z[j]);
              rSP[j](1) = cos(k[j]*2*M_PI/z[j]);
              rSP[j] = gear[j]->getFrame()->getOrientation()*rSP[j];
              double cdel = signj*(rSP[j].T()*rS1S2/a);
              del[j] = signi*signj*(gear[j]->getFrame()->getOrientation().col(2).T()*crossProduct(rS1S2,rSP[j])>=0?acos(cdel):-acos(cdel));
              eta[j] = ga[j] + del[j] + al;
              rSP[j](0) = signi*rb[j]*(sin(eta[j])-cos(eta[j])*eta[j]);
              rSP[j](1) = rb[j]*(cos(eta[j])+sin(eta[j])*eta[j]);
              rSP[j] = gear[j]->getFrame()->getOrientation()*BasicRotAIKz(signi*ga[j]+k[j]*2*M_PI/z[j])*rSP[j];
              rOP[j] = gear[j]->getFrame()->getPosition() + rSP[j];
            }

            Vec3 n1(NONINIT);
            n1(0) = -signi*cos(eta[0]);
            n1(1) = sin(eta[0]);
            n1(2) = -signi*cos(al)*tan(beta[0]);
            n1 /= sqrt(1+pow(cos(al)*tan(beta[0]),2));
            n1 = gear[0]->getFrame()->getOrientation()*BasicRotAIKz(signi*ga[0]+k[0]*2*M_PI/z[0])*n1;

            Vec3 u1(NONINIT);
            u1(0) = signi*sin(eta[0]);
            u1(1) = cos(eta[0]);
            u1(2) = 0;
            u1 = gear[0]->getFrame()->getOrientation()*BasicRotAIKz(signi*ga[0]+k[0]*2*M_PI/z[0])*u1;

            double g = n1.T()*(rOP[1]-rOP[0]);
            if(g>-0.5*M_PI*d0[0]/z[0] and g<contact.getGeneralizedRelativePosition(false)(0)) {
              ksave[ii][0] = k[0];
              ksave[ii][1] = k[1];
              etasave[ii][0] = eta[0];
              etasave[ii][1] = eta[1];
              signisave[ii] = signi;
              delsave = del[0];

              contact.getContourFrame(igear[0])->setPosition(rOP[0]);
              contact.getContourFrame(igear[0])->getOrientation(false).set(0,n1);
              contact.getContourFrame(igear[0])->getOrientation(false).set(1,u1);
              contact.getContourFrame(igear[0])->getOrientation(false).set(2,crossProduct(n1,contact.getContourFrame(igear[0])->getOrientation(false).col(1)));

              contact.getContourFrame(igear[1])->setPosition(rOP[1]);
              contact.getContourFrame(igear[1])->getOrientation(false).set(0, -contact.getContourFrame(igear[0])->getOrientation(false).col(0));
              contact.getContourFrame(igear[1])->getOrientation(false).set(1, -contact.getContourFrame(igear[0])->getOrientation(false).col(1));
              contact.getContourFrame(igear[1])->getOrientation(false).set(2, contact.getContourFrame(igear[0])->getOrientation(false).col(2));

              contact.getGeneralizedRelativePosition(false)(0) = g;
            }
          }
        }
      }
    }
  }

  void ContactKinematicsCylindricalGearCylindricalGear::updatewb(SingleContact &contact, int ii) {
    const Vec3 n1 = contact.getContourFrame(igear[0])->evalOrientation().col(0);
    const Vec3 vC1 = contact.getContourFrame(igear[0])->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(igear[1])->evalVelocity();
    const Vec3 u1 = contact.getContourFrame(igear[0])->evalOrientation().col(1);
    const Vec3 u2 = contact.getContourFrame(igear[1])->evalOrientation().col(1);
    Vec3 R[2], N1, U2;
    for(int j=0; j<2; j++) {
      R[j](0) = signisave[ii]*rb[j]*sin(etasave[ii][j])*etasave[ii][j];
      R[j](1) = rb[j]*cos(etasave[ii][j])*etasave[ii][j];
      R[j] = gear[j]->getFrame()->getOrientation()*BasicRotAIKz(signisave[ii]*ga[j]+ksave[ii][j]*2*M_PI/z[j])*R[j];
    }
    N1(0) = signisave[ii]*sin(etasave[ii][0])/sqrt(1+pow(cos(al)*tan(beta[0]),2));
    N1(1) = cos(etasave[ii][0])/sqrt(1+pow(cos(al)*tan(beta[0]),2));
    N1 = gear[0]->getFrame()->getOrientation()*BasicRotAIKz(signisave[ii]*ga[0]+ksave[ii][0]*2*M_PI/z[0])*N1;
    int sign2 = gear[1]->getExternalToothed()?1:-1;
    U2(0) = signisave[ii]*sign2*cos(etasave[ii][1]);
    U2(1) = -sign2*sin(etasave[ii][1]);
    U2 = gear[1]->getFrame()->getOrientation()*BasicRotAIKz(signisave[ii]*ga[1]+ksave[ii][1]*2*M_PI/z[1])*U2;
    const Vec3 parnPart1 = crossProduct(gear[0]->getFrame()->evalAngularVelocity(),n1);
    const Vec3 paruPart2 = crossProduct(gear[1]->getFrame()->evalAngularVelocity(),u2);
    const Vec3 parWvCParZeta1 = crossProduct(gear[0]->getFrame()->evalAngularVelocity(),R[0]);
    const Vec3 parWvCParZeta2 = crossProduct(gear[1]->getFrame()->evalAngularVelocity(),R[1]);

    SqrMat A(2,NONINIT);
    A(0,0)=-u1.T()*R[0].col(0);
    A(0,1)=u1.T()*R[1].col(0);
    A(1,0)=u2.T()*N1.col(0);
    A(1,1)=n1.T()*U2.col(0);

    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-u2.T()*parnPart1-n1.T()*paruPart2;

    Vec zetad = slvLU(A,b);

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += (N1*zetad(0)+parnPart1).T()*(vC2-vC1)+n1.T()*(parWvCParZeta2*zetad(1)-parWvCParZeta1*zetad(0));
    if(contact.isTangentialForceLawSetValuedAndActive())
      throw runtime_error("Tangential force law must be single valued for gear to gear contacts");
  }

}
