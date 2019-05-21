/* Copyright (C) 2004-2019 MBSim Development Team
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
#include "cylindricalgear_rack.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/cylindrical_gear.h"
#include "mbsim/contours/rack.h"
#include <mbsim/utils/rotarymatrices.h>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCylindricalGearRack::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CylindricalGear*>(contour[0])) {
      igear = 0; irack = 1;
      gear = static_cast<CylindricalGear*>(contour[0]);
      rack = static_cast<Rack*>(contour[1]);
    }
    else {
      igear = 1; irack = 0;
      gear = static_cast<CylindricalGear*>(contour[1]);
      rack = static_cast<Rack*>(contour[0]);
    }
    m = gear->getModule()/cos(gear->getHelixAngle());
    al0 = atan(tan(gear->getPressureAngle())/cos(gear->getHelixAngle()));
    double phi0 = tan(al0)-al0;
    double s0 = m*M_PI/2;
    double dk[2];
    z[0] = gear->getNumberOfTeeth();
    d0[0] = m*z[0];
    db[0] = d0[0]*cos(al0);
    dk[0] = d0[0]+2*m;
    rb[0] = db[0]/2;
    sb[0] = db[0]*(s0/d0[0]+phi0)-gear->getBacklash()*cos(al0);
    ga[0] = sb[0]/rb[0]/2;
    beta[0] = gear->getHelixAngle();
    beta[1] = rack->getHelixAngle();
    delmin[0] = -al0-ga[0];
    delmax[0] = tan(acos(db[0]/dk[0]))-al0-ga[0];
    delmin[1] = -m/cos(al);
    delmax[1] = m/cos(al);
    z[1] = rack->getNumberOfTeeth();
  }

  void ContactKinematicsCylindricalGearRack::updateg(SingleContact &contact, int ii) {
    contact.getGeneralizedRelativePosition(false)(0) = 1e10;
    double eta[2], del[2];
    al = al0;
    Vec3 r = gear->getFrame()->evalPosition() - rack->getFrame()->evalPosition();
    double rs = rack->getFrame()->getOrientation().col(0).T()*r;
    double rp = rack->getFrame()->getOrientation().col(1).T()*r;
    double s0h = (m*M_PI/2-rack->getBacklash())/2;
    for(int i=0; i<2; i++) {
      int signi = i?-1:1;
      Vec3 rOP[2], rSP[2];
      vector<int> v[2];
      if(maxNumContacts==1) {
        int k = 0;
        double rsi = 1e10;
        for(int k_=0; k_<z[1]; k_++) {
          double rsi_ = fabs(rs - k_*M_PI*m + signi*s0h);
          if(rsi_<rsi) {
            rsi = rsi_;
            k = k_;
          }
        }
        v[1].push_back(k);

        Vec3 rP1S2 = rack->getFrame()->getPosition() + rack->getFrame()->getOrientation().col(0)*(k*M_PI*m-signi*s0h) - gear->getFrame()->getPosition();
        double cdel = 0;
        k = 0;
        for(int k_=0; k_<z[0]; k_++) {
          double ep = k_*2*M_PI/z[0]+signi*ga[0];
          rSP[0](0) = -sin(ep);
          rSP[0](1) = cos(ep);
          rSP[0] = gear->getFrame()->getOrientation()*rSP[0];
          double cdel_ = rSP[0].T()*rP1S2/nrm2(rP1S2);
          if(cdel_>cdel) {
            cdel = cdel_;
            k = k_;
          }
        }
        v[0].push_back(k);
      }
      else {
        for(int k_=0; k_<z[1]; k_++) {
          double rsi = rs - k_*M_PI*m;
          double rpi = rp - d0[0]/2;
          double l = sin(al)*(s0h+signi*rsi)+rpi*cos(al); 
          if(l>delmin[1] and l<delmax[1])
            v[1].push_back(k_);
        }
        for(int k_=0; k_<z[0]; k_++) {
          double ep = k_*2*M_PI/z[0];
          rSP[0](0) = -sin(ep);
          rSP[0](1) = cos(ep);
          rSP[0] = gear->getFrame()->getOrientation()*rSP[0];
          double cdel = -(rSP[0].T()*rack->getFrame()->getOrientation().col(1));
          double del = signi*(gear->getFrame()->getOrientation().col(2).T()*crossProduct(rack->getFrame()->getOrientation().col(1),rSP[0])>=0?-acos(cdel):acos(cdel));
          if(del>delmin[0] and del<delmax[0])
            v[0].push_back(k_);
        }
      }

      double k[2];
      for (auto & i0 : v[0]) {
        for (auto & i1 : v[1]) {
          k[0] = i0;
          k[1] = i1;
          if(ii==0 or not(k[0]==ksave[0][0] and k[1]==ksave[0][1])) {
            double rsi = rs - k[1]*M_PI*m;
            double rpi = rp - d0[0]/2;
            double l = sin(al)*(s0h+signi*rsi)+rpi*cos(al); 
            rSP[1](0) = signi*l*sin(al);
            rSP[1](1) = l*cos(al);
            rSP[1] = rack->getFrame()->getOrientation()*rSP[1];
            rOP[1] = rack->getFrame()->getPosition() + rack->getFrame()->getOrientation().col(0)*(k[1]*M_PI*m-signi*s0h) + rSP[1];

            rSP[0](0) = -sin(k[0]*2*M_PI/z[0]);
            rSP[0](1) = cos(k[0]*2*M_PI/z[0]);
            rSP[0] = gear->getFrame()->getOrientation()*rSP[0];
            double cdel = -(rSP[0].T()*rack->getFrame()->getOrientation().col(1));
            del[0] = signi*(gear->getFrame()->getOrientation().col(2).T()*crossProduct(rack->getFrame()->getOrientation().col(1),rSP[0])>=0?-acos(cdel):acos(cdel));
            eta[0] = ga[0] + del[0] + al;
            rSP[0](0) = signi*rb[0]*(sin(eta[0])-cos(eta[0])*eta[0]);
            rSP[0](1) = rb[0]*(cos(eta[0])+sin(eta[0])*eta[0]);
            rSP[0] = gear->getFrame()->getOrientation()*BasicRotAIKz(signi*ga[0]+k[0]*2*M_PI/z[0])*rSP[0];
            rOP[0] = gear->getFrame()->getPosition() + rSP[0];

            Vec3 n2(NONINIT);
            n2(0) = -signi*cos(al);
            n2(1) = sin(al);
            n2(2) = -signi*cos(al)*tan(beta[1]);
            n2 /= sqrt(1+pow(cos(al)*tan(beta[1]),2));
            n2 = rack->getFrame()->getOrientation()*n2;

            Vec3 u2(NONINIT);
            u2(0) = signi*sin(al);
            u2(1) = cos(al);
            u2(2) = 0;
            u2 = rack->getFrame()->getOrientation()*u2;

            double g = n2.T()*(rOP[0]-rOP[1]);
            if(g>-0.5*M_PI*d0[0]/z[0] and g<contact.getGeneralizedRelativePosition(false)(0)) {
              ksave[ii][0] = k[0];
              ksave[ii][1] = k[1];
              etasave[ii] = eta[0];
              signisave[ii] = signi;
              delsave = del[0];

              contact.getContourFrame(irack)->setPosition(rOP[1]);
              contact.getContourFrame(irack)->getOrientation(false).set(0,n2);
              contact.getContourFrame(irack)->getOrientation(false).set(1,u2);
              contact.getContourFrame(irack)->getOrientation(false).set(2,crossProduct(n2,contact.getContourFrame(irack)->getOrientation(false).col(1)));

              contact.getContourFrame(igear)->setPosition(rOP[0]);
              contact.getContourFrame(igear)->getOrientation(false).set(0, -contact.getContourFrame(irack)->getOrientation(false).col(0));
              contact.getContourFrame(igear)->getOrientation(false).set(1, -contact.getContourFrame(irack)->getOrientation(false).col(1));
              contact.getContourFrame(igear)->getOrientation(false).set(2, contact.getContourFrame(irack)->getOrientation(false).col(2));

              contact.getGeneralizedRelativePosition(false)(0) = g;
            }
          }
        }
      }
    }
  }

  void ContactKinematicsCylindricalGearRack::updatewb(SingleContact &contact, int ii) {
    const Vec3 n1 = contact.getContourFrame(igear)->evalOrientation().col(0);
    const Vec3 vC1 = contact.getContourFrame(igear)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(irack)->evalVelocity();
    const Vec3 u1 = contact.getContourFrame(igear)->evalOrientation().col(1);
    const Vec3 u2 = contact.getContourFrame(irack)->evalOrientation().col(1);
    Vec3 R[2], N1;

    R[0](0) = signisave[ii]*rb[0]*sin(etasave[ii])*etasave[ii];
    R[0](1) = rb[0]*cos(etasave[ii])*etasave[ii];
    R[0] = gear->getFrame()->getOrientation()*BasicRotAIKz(signisave[ii]*ga[0]+ksave[ii][0]*2*M_PI/z[0])*R[0];

    R[1](0) = signisave[ii]*sin(al);
    R[1](1) = cos(al);
    R[1] = rack->getFrame()->getOrientation()*R[1];

    N1(0) = signisave[ii]*sin(etasave[ii])/sqrt(1+pow(cos(al)*tan(beta[0]),2));
    N1(1) = cos(etasave[ii])/sqrt(1+pow(cos(al)*tan(beta[0]),2));
    N1 = gear->getFrame()->getOrientation()*BasicRotAIKz(signisave[ii]*ga[0]+ksave[ii][0]*2*M_PI/z[0])*N1;

    const Vec3 parnPart1 = crossProduct(gear->getFrame()->evalAngularVelocity(),n1);
    const Vec3 paruPart2 = crossProduct(rack->getFrame()->evalAngularVelocity(),u2);
    const Vec3 parWvCParZeta1 = crossProduct(gear->getFrame()->evalAngularVelocity(),R[0]);
    const Vec3 parWvCParZeta2 = crossProduct(rack->getFrame()->evalAngularVelocity(),R[1]);

    SqrMat A(2,NONINIT);
    A(0,0)=-u1.T()*R[0].col(0);
    A(0,1)=u1.T()*R[1].col(0);
    A(1,0)=u2.T()*N1.col(0);
    A(1,1)=0;

    Vec b(2,NONINIT);
    b(0)=-u1.T()*(vC2-vC1);
    b(1)=-u2.T()*parnPart1-n1.T()*paruPart2;

    Vec zetad = slvLU(A,b);

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += (N1*zetad(0)+parnPart1).T()*(vC2-vC1)+n1.T()*(parWvCParZeta2*zetad(1)-parWvCParZeta1*zetad(0));
    if(contact.isTangentialForceLawSetValuedAndActive())
      throw runtime_error("Tangential force law must be single valued for gear wheel to gear wheel contacts");
  }

}
