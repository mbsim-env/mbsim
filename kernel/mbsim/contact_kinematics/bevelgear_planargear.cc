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
#include "bevelgear_planargear.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/bevel_gear.h"
#include "mbsim/contours/planar_gear.h"
#include <mbsim/utils/rotarymatrices.h>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsBevelGearPlanarGear::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<BevelGear*>(contour[0])) {
      ibevelgear = 0; iplanargear = 1;
      bevelgear = static_cast<BevelGear*>(contour[0]);
      planargear = static_cast<PlanarGear*>(contour[1]);
    }
    else {
      ibevelgear = 1; iplanargear = 0;
      bevelgear = static_cast<BevelGear*>(contour[1]);
      planargear = static_cast<PlanarGear*>(contour[0]);
    }
    beta[0] = bevelgear->getHelixAngle();
    beta[1] = planargear->getHelixAngle();
    m = bevelgear->getModule()/cos(beta[0]);
    al0 = bevelgear->getPressureAngle();
    z[0] = bevelgear->getNumberOfTeeth();
    z[1] = planargear->getNumberOfTeeth();
    delh2 = (M_PI/2-planargear->getBacklash()/m)/z[1];
    delh1 = (M_PI/2-bevelgear->getBacklash()/m)/z[0];
  }

  void ContactKinematicsBevelGearPlanarGear::updateg(SingleContact &contact, int ii) {
    contact.getGeneralizedRelativePosition(false)(0) = 1e10;
    Vec3 ey1 = bevelgear->getFrame()->evalOrientation().T()*planargear->getFrame()->evalOrientation().col(1);
    Vec3 ez2 = planargear->getFrame()->getOrientation().T()*bevelgear->getFrame()->getOrientation().col(2);
    double phi1 = (ey1(0)>=0?1:-1)*acos(ey1(1)/sqrt(pow(ey1(0),2)+pow(ey1(1),2))); 
    double phi2 = (ez2(0)>=0?-1:1)*acos(ez2(2)/sqrt(pow(ez2(0),2)+pow(ez2(2),2))); 
    if(nrm2(planargear->getFrame()->evalPosition()-bevelgear->getFrame()->evalPosition()+m*z[0]/2/tan(bevelgear->getPitchAngle())*bevelgear->getFrame()->getOrientation().col(2))>1e-8)
       msg(Warn)<<"Large devitation detected at t="<<planargear->getTime()<<"\nContact kinematics may be wrong!" <<endl;

    for(int i=0; i<2; i++) {
      int signi = i?-1:1;
      Vec3 rOP[2];
      vector<int> v[2];
      if(maxNumContacts==1) {
        v[1].push_back(round(-(phi2 + signi*delh2)/(2*M_PI/z[1])));
        double phi1corr = (phi2 + v[1][0]*2*M_PI/z[1] + signi*delh2)*z[1]/z[0];
        v[0].push_back(round(-(phi1 - signi*delh1 - phi1corr)/(2*M_PI/z[0])));
      }
      else {
        int kmax = floor((bevelgear->getPhiMaxHigh(i) - (phi1 - signi*delh1))/(2*M_PI/z[0]));
        int kmin = ceil((bevelgear->getPhiMinHigh(i) - (phi1 - signi*delh1))/(2*M_PI/z[0]));
        for(int k_=kmin; k_<=kmax; k_++)
          v[0].push_back(k_);
        kmax = floor((planargear->getPhiMaxHigh(i) - (phi2 + signi*delh2))/(2*M_PI/z[1]));
        kmin = ceil((planargear->getPhiMinHigh(i) - (phi2 + signi*delh2))/(2*M_PI/z[1]));
        for(int k_=kmin; k_<=kmax; k_++)
          v[1].push_back(k_);
      }

      double k[2];
      for (auto & i0 : v[0]) {
        for (auto & i1 : v[1]) {
          k[0] = i0;
          k[1] = i1;
          if(ii==0 or not(k[0]==ksave[0][0] and k[1]==ksave[0][1])) {
            Vec2 zeta1(NONINIT), zeta2(NONINIT);
            double phi1q = phi1+k[0]*2*M_PI/z[0]-signi*delh1;
            zeta1(0) = -phi1q;
            double s1 = 0;
            if(phi1q>bevelgear->getPhiMaxLow(i))
              s1 = bevelgear->getSPhiMaxHigh(i)/(bevelgear->getPhiMaxHigh(i)-bevelgear->getPhiMaxLow(i))*(phi1q-bevelgear->getPhiMaxLow(i));
            else if(phi1q<bevelgear->getPhiMinLow(i))
              s1 = bevelgear->getSPhiMinHigh(i)/(bevelgear->getPhiMinHigh(i)-bevelgear->getPhiMinLow(i))*(phi1q-bevelgear->getPhiMinLow(i));
            double phi2q = phi2+k[1]*2*M_PI/z[1]+signi*delh2;
            double s2 = 0;
            if(phi2q>planargear->getPhiMaxLow(i))
              s2 = planargear->getSPhiMaxHigh(i)/(planargear->getPhiMaxHigh(i)-planargear->getPhiMaxLow(i))*(phi2q-planargear->getPhiMaxLow(i));
            else if(phi2q<planargear->getPhiMinLow(i))
              s2 = planargear->getSPhiMinHigh(i)/(planargear->getPhiMinHigh(i)-planargear->getPhiMinLow(i))*(phi2q-planargear->getPhiMinLow(i));
            double s = fabs(s2)>fabs(s1)?s2:s1;
            zeta2(1) = (s*cos(phi2q+beta[1])-m*z[1]/2*sin(phi2q)*pow(sin(al0),2)*sin(beta[1]))/(sin(phi2q+beta[1])*pow(sin(al0),2)*sin(beta[1])+cos(phi2q+beta[1])*cos(beta[1]));
            zeta2(0) = (sin(phi2q)/cos(phi2q+beta[1])*m*z[1]/2 + zeta2(1)*tan(phi2q+beta[1]))*sin(al0);
            planargear->setFlank(signi);
            planargear->setTooth(k[1]);
            rOP[1] = planargear->evalPosition(zeta2);
            phi2q = -double(z[0])/z[1]*zeta1(0);
            zeta1(1) = (s*cos(phi2q-beta[0])+m*z[1]/2*sin(phi2q)*pow(sin(al0),2)*sin(beta[0]))/(-sin(phi2q-beta[0])*pow(sin(al0),2)*sin(beta[0])+cos(phi2q-beta[0])*cos(beta[0]));
            bevelgear->setFlank(signi);
            bevelgear->setTooth(k[0]);
            rOP[0] = bevelgear->evalPosition(zeta1);

            Vec n2 = planargear->evalWn(zeta2);

            double g = n2.T()*(rOP[0]-rOP[1]);
            if(g>-0.5*M_PI*m and g<contact.getGeneralizedRelativePosition(false)(0)) {
              ksave[ii][0] = k[0];
              ksave[ii][1] = k[1];
              signisave[ii] = signi;

              contact.getContourFrame(iplanargear)->setZeta(zeta2);
              contact.getContourFrame(iplanargear)->setPosition(rOP[1]);
              contact.getContourFrame(iplanargear)->getOrientation(false).set(0,n2);
              contact.getContourFrame(iplanargear)->getOrientation(false).set(1,planargear->evalWu(zeta2));
              contact.getContourFrame(iplanargear)->getOrientation(false).set(2,crossProduct(contact.getContourFrame(iplanargear)->getOrientation(false).col(0),contact.getContourFrame(iplanargear)->getOrientation(false).col(1)));

              contact.getContourFrame(ibevelgear)->setZeta(zeta1);
              contact.getContourFrame(ibevelgear)->setPosition(rOP[0]);
              contact.getContourFrame(ibevelgear)->getOrientation(false).set(0,bevelgear->evalWn(zeta1));
              contact.getContourFrame(ibevelgear)->getOrientation(false).set(1,bevelgear->evalWu(zeta1));
              contact.getContourFrame(ibevelgear)->getOrientation(false).set(2,crossProduct(contact.getContourFrame(ibevelgear)->getOrientation(false).col(0),contact.getContourFrame(ibevelgear)->getOrientation(false).col(1)));

              contact.getGeneralizedRelativePosition(false)(0) = g;
            }
          }
        }
      }
    }
  }

  void ContactKinematicsBevelGearPlanarGear::updatewb(SingleContact &contact, int ii) {
    const Vec3 n1 = contact.getContourFrame(iplanargear)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(iplanargear)->evalOrientation().col(1);
    const Vec3 u2 = contact.getContourFrame(ibevelgear)->evalOrientation().col(1);
    const Vec3 vC1 = contact.getContourFrame(iplanargear)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(ibevelgear)->evalVelocity();
    const Vec3 parnPart1 = crossProduct(planargear->getFrame()->getAngularVelocity(),n1);
    const Vec3 paruPart2 = crossProduct(bevelgear->getFrame()->getAngularVelocity(),u2);

    planargear->setFlank(signisave[ii]);
    planargear->setTooth(ksave[ii][1]);
    Vec3 R1 = planargear->evalWs(contact.getContourFrame(iplanargear)->getZeta());

    bevelgear->setFlank(signisave[ii]);
    bevelgear->setTooth(ksave[ii][0]);
    Vec3 R2 = bevelgear->evalWs(contact.getContourFrame(ibevelgear)->getZeta());
    Vec3 U2 = bevelgear->evalParDer1Wu(contact.getContourFrame(ibevelgear)->getZeta());

    Vec3 parWvCParZeta1 = crossProduct(planargear->getFrame()->getAngularVelocity(),R1);
    Vec3 parWvCParZeta2 = crossProduct(bevelgear->getFrame()->getAngularVelocity(),R2);

    SqrMat A(2,NONINIT);
    A(0,0) = -u1.T()*R1;
    A(0,1) = u1.T()*R2;
    A(1,0) = 0;
    A(1,1) = n1.T()*U2;

    Vec b_(2,NONINIT);
    b_(0) = -u1.T()*(vC2-vC1);
    b_(1) = -u2.T()*parnPart1-n1.T()*paruPart2;
    Vec zetad = slvLU(A,b_);

    if(contact.isNormalForceLawSetValued())
      contact.getwb(false)(0) += parnPart1.T()*(vC2-vC1)+n1.T()*(parWvCParZeta2*zetad(1)-parWvCParZeta1*zetad(0));
    if(contact.isTangentialForceLawSetValuedAndActive())
      throw runtime_error("Tangential force law must be single valued for gear to gear contacts");
  }

}
