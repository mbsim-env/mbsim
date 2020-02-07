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
    beta[0] = gear->getHelixAngle();
    beta[1] = rack->getHelixAngle();
    m = gear->getModule()/cos(beta[0]);
    al0 = gear->getPressureAngle();
    double al = atan(tan(gear->getPressureAngle())/cos(gear->getHelixAngle()));
    z[0] = gear->getNumberOfTeeth();
    z[1] = rack->getNumberOfTeeth();
    s0h2 = (m*M_PI/2-rack->getBacklash())/2;
    delh1 = (M_PI/2-gear->getBacklash()/m)/z[0];
    etamax1[0][0] = gear->getEtaMax(m*z[0]*cos(al)/2,(beta[0]>=0?-1:1)*gear->getWidth()/2);
    etamax1[1][0] = gear->getEtaMax(m*z[0]*cos(al)/2,(beta[0]>=0?1:-1)*gear->getWidth()/2);
    etamax1[0][1] = gear->getEtaMax(m*z[0]/2+gear->getModule(),(beta[0]>=0?1:-1)*gear->getWidth()/2);
    etamax1[1][1] = gear->getEtaMax(m*z[0]/2+gear->getModule(),(beta[0]>=0?-1:1)*gear->getWidth()/2);
  }

  void ContactKinematicsCylindricalGearRack::updateg(SingleContact &contact, int ii) {
    contact.getGeneralizedRelativePosition(false)(0) = 1e10;
    Vec3 r = rack->getFrame()->evalPosition()-gear->getFrame()->evalPosition();
    double x2 = rack->getFrame()->getOrientation().col(0).T()*r;
    double y2 = rack->getFrame()->getOrientation().col(1).T()*r+m*z[0]/2;
    double z2 = rack->getFrame()->getOrientation().col(2).T()*r/2;
    double z1 = gear->getFrame()->getOrientation().col(2).T()*r/2;
    Vec3 ey1 = gear->getFrame()->evalOrientation().T()*rack->getFrame()->evalOrientation().col(1);
    double phi1 = (ey1(0)>=0?1:-1)*acos(ey1(1)/sqrt(pow(ey1(0),2)+pow(ey1(1),2)));
    double xmax[2][2];
    double xmax0 = fabs((-rack->getModule()/cos(al0)*(pow(sin(beta[1])*sin(al0),2)+pow(cos(beta[1]),2))/cos(beta[1])+y2*cos(al0)*cos(beta[1]))/sin(al0)+z2*tan(beta[1]));
    xmax[0][0] = (beta[1]>=0?-1:1)*rack->getWidth()/2*tan(beta[1])+xmax0;
    xmax[1][0] = (beta[1]>=0?1:-1)*rack->getWidth()/2*tan(beta[1])+xmax0;
    xmax0 = fabs((rack->getModule()/cos(al0)*(pow(sin(beta[1])*sin(al0),2)+pow(cos(beta[1]),2))/cos(beta[1])+y2*cos(al0)*cos(beta[1]))/sin(al0)+z2*tan(beta[1]));
    xmax[0][1] = (beta[1]>=0?-1:1)*rack->getWidth()/2*tan(beta[1])+xmax0;
    xmax[1][1] = (beta[1]>=0?1:-1)*rack->getWidth()/2*tan(beta[1])+xmax0;
    for(int i=0; i<2; i++) {
      int signi = i?-1:1;
      Vec3 rOP[2];
      vector<int> v[2];
      if(maxNumContacts==1) {
        v[1].push_back(round(-(x2 + signi*s0h2)/(m*M_PI)));
        double phi1corr = (x2 + v[1][0]*M_PI*m + signi*s0h2)/(m*z[0]/2);
        v[0].push_back(round(-(phi1 - signi*delh1 - phi1corr)/(2*M_PI/z[0])));
      }
      else {
        int kmax = floor(-(-etamax1[1][i] + (phi1 - signi*delh1))/(2*M_PI/z[0]));
        int kmin = ceil(-(etamax1[1][not i] + (phi1 - signi*delh1))/(2*M_PI/z[0]));
        for(int k_=kmin; k_<=kmax; k_++)
          v[0].push_back(k_);
        kmax = floor((xmax[1][i] - (x2 + signi*s0h2))/(m*M_PI));
        kmin = ceil((-xmax[1][not i] - (x2 + signi*s0h2))/(m*M_PI));
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
            zeta1(0) = -(phi1+k[0]*2*M_PI/z[0]-signi*delh1);
            double s = 0;
            if(zeta1(0)>etamax1[0][not i])
              s = (beta[0]>=0?-1:1)*max(s,gear->getWidth()/2/(etamax1[1][not i]-etamax1[0][not i])*(zeta1(0)-etamax1[0][not i]));
            else if(zeta1(0)<-etamax1[0][i])
              s = (beta[0]>=0?1:-1)*max(s,gear->getWidth()/2/(-etamax1[1][i]+etamax1[0][i])*(zeta1(0)+etamax1[0][i]));
            double x2q = (x2+k[1]*M_PI*m+signi*s0h2);
            if(x2q>xmax[0][i])
              s = (beta[1]>=0?-1:1)*max(fabs(s),rack->getWidth()/2/(xmax[1][i]-xmax[0][i])*(x2q-xmax[0][i]));
            else if(x2q<-xmax[0][not i])
              s = (beta[1]>=0?1:-1)*max(fabs(s),rack->getWidth()/2/(-xmax[1][not i]+xmax[0][not i])*(x2q+xmax[0][not i]));
            zeta2(1) = ((signi*y2*cos(al0)*cos(beta[1])-x2q*sin(al0))*sin(al0)*sin(beta[1])+(s-z2)*cos(beta[1]))/(pow(sin(beta[1])*sin(al0),2)+pow(cos(beta[1]),2));
            zeta2(0) = (x2q/cos(beta[1])+zeta2(1)*tan(beta[1]))*sin(al0)-signi*y2*cos(al0);
            rack->setFlank(signi);
            rack->setTooth(k[1]);
            rOP[1] = rack->evalPosition(zeta2);
            zeta1(1) = (-m*z[0]/2*zeta1(0)*pow(sin(al0),2)*sin(beta[0])+(s+z1)*cos(beta[0]))/(pow(sin(beta[0])*sin(al0),2)+pow(cos(beta[0]),2));
            gear->setFlank(signi);
            gear->setTooth(k[0]);
            rOP[0] = gear->evalPosition(zeta1);

            Vec n2 = rack->evalWn(zeta2);

            double g = n2.T()*(rOP[0]-rOP[1]);
            if(g>-0.5*M_PI*m and g<contact.getGeneralizedRelativePosition(false)(0)) {
              ksave[ii][0] = k[0];
              ksave[ii][1] = k[1];
              signisave[ii] = signi;

              contact.getContourFrame(irack)->setZeta(zeta2);
              contact.getContourFrame(irack)->setPosition(rOP[1]);
              contact.getContourFrame(irack)->getOrientation(false).set(0,n2);
              contact.getContourFrame(irack)->getOrientation(false).set(1,rack->evalWu(zeta2));
              contact.getContourFrame(irack)->getOrientation(false).set(2,crossProduct(contact.getContourFrame(irack)->getOrientation(false).col(0),contact.getContourFrame(irack)->getOrientation(false).col(1)));

              contact.getContourFrame(igear)->setZeta(zeta1);
              contact.getContourFrame(igear)->setPosition(rOP[0]);
              contact.getContourFrame(igear)->getOrientation(false).set(0,gear->evalWn(zeta1));
              contact.getContourFrame(igear)->getOrientation(false).set(1,gear->evalWu(zeta1));
              contact.getContourFrame(igear)->getOrientation(false).set(2,crossProduct(contact.getContourFrame(igear)->getOrientation(false).col(0),contact.getContourFrame(igear)->getOrientation(false).col(1)));

              contact.getGeneralizedRelativePosition(false)(0) = g;
            }
          }
        }
      }
    }
  }

  void ContactKinematicsCylindricalGearRack::updatewb(SingleContact &contact, int ii) {
    const Vec3 n1 = contact.getContourFrame(irack)->evalOrientation().col(0);
    const Vec3 u1 = contact.getContourFrame(irack)->evalOrientation().col(1);
    const Vec3 u2 = contact.getContourFrame(igear)->evalOrientation().col(1);
    const Vec3 vC1 = contact.getContourFrame(irack)->evalVelocity();
    const Vec3 vC2 = contact.getContourFrame(igear)->evalVelocity();
    const Vec3 parnPart1 = crossProduct(rack->getFrame()->getAngularVelocity(),n1);
    const Vec3 paruPart2 = crossProduct(gear->getFrame()->getAngularVelocity(),u2);

    rack->setFlank(signisave[ii]);
    rack->setTooth(ksave[ii][1]);
    Vec3 R1 = rack->evalWs(contact.getContourFrame(irack)->getZeta());

    gear->setFlank(signisave[ii]);
    gear->setTooth(ksave[ii][0]);
    Vec3 R2 = gear->evalWs(contact.getContourFrame(igear)->getZeta());
    Vec3 U2 = gear->evalParDer1Wu(contact.getContourFrame(igear)->getZeta());

    Vec3 parWvCParZeta1 = crossProduct(rack->getFrame()->getAngularVelocity(),R1);
    Vec3 parWvCParZeta2 = crossProduct(gear->getFrame()->getAngularVelocity(),R2);

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
