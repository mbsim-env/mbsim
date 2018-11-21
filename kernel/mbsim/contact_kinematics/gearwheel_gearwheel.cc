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
#include "gearwheel_gearwheel.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/gear_wheel.h"
#include <mbsim/utils/rotarymatrices.h>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsGearWheelGearWheel::assignContours(const vector<Contour*> &contour) {
    igearwheel[0] = 0; igearwheel[1] = 1;
    m = static_cast<GearWheel*>(contour[0])->getModule();
    al0 = static_cast<GearWheel*>(contour[0])->getPressureAngle();
    double phi0 = tan(al0)-al0;
    double s0 = m*M_PI/2;
    for(int i=0; i<2; i++) {
      gearwheel[i] = static_cast<GearWheel*>(contour[i]);
      z[i] = gearwheel[i]->getNumberOfTeeth();
      d0[i] = m*z[i];
      db[i] = d0[i]*cos(al0);
      rb[i] = db[i]/2;
      sb[i] = db[i]*(s0/d0[i]+phi0);
      ga[i] = sb[i]/rb[i]/2;
    }
    a0 = (d0[0]+d0[1])/2;
    maxNumContacts = 2;
  }

  void ContactKinematicsGearWheelGearWheel::updateg(SingleContact &contact, int i) {
    int signi = i?-1:1;
    Vec3 rS1S2 = gearwheel[1]->getFrame()->evalPosition() - gearwheel[0]->getFrame()->evalPosition();
    a = nrm2(rS1S2);
    al = acos(a0/a*cos(al0));
    Vec3 n1 = rS1S2/a;
    Vec3 axis = gearwheel[0]->getFrame()->getOrientation().col(2);
    double phi = signi*(M_PI/2-al);
    const double cosq=cos(phi);
    const double sinq=sin(phi);
    const double onemcosq=1-cosq;
    const double a0a1=axis.e(0)*axis.e(1);
    const double a0a2=axis.e(0)*axis.e(2);
    const double a1a2=axis.e(1)*axis.e(2);
    SqrMat3 A(NONINIT);
    A.e(0,0) = cosq+onemcosq*axis.e(0)*axis.e(0);
    A.e(1,0) = onemcosq*a0a1+axis.e(2)*sinq;
    A.e(2,0) = onemcosq*a0a2-axis.e(1)*sinq;
    A.e(0,1) = onemcosq*a0a1-axis.e(2)*sinq;
    A.e(1,1) = cosq+onemcosq*axis.e(1)*axis.e(1);
    A.e(2,1) = onemcosq*a1a2+axis.e(0)*sinq;
    A.e(0,2) = onemcosq*a0a2+axis.e(1)*sinq;
    A.e(1,2) = onemcosq*a1a2-axis.e(0)*sinq;
    A.e(2,2) = cosq+onemcosq*axis.e(2)*axis.e(2);
    n1 = A*n1;

    contact.getContourFrame(igearwheel[0])->getOrientation(false).set(0,n1);
    contact.getContourFrame(igearwheel[0])->getOrientation(false).set(1,crossProduct(gearwheel[0]->getFrame()->getOrientation().col(2),n1));
    contact.getContourFrame(igearwheel[0])->getOrientation(false).set(2,crossProduct(n1,contact.getContourFrame(igearwheel[0])->getOrientation(false).col(1)));

    contact.getContourFrame(igearwheel[1])->getOrientation(false).set(0, -contact.getContourFrame(igearwheel[0])->getOrientation(false).col(0));
    contact.getContourFrame(igearwheel[1])->getOrientation(false).set(1, -contact.getContourFrame(igearwheel[0])->getOrientation(false).col(1));
    contact.getContourFrame(igearwheel[1])->getOrientation(false).set(2, contact.getContourFrame(igearwheel[0])->getOrientation(false).col(2));

    double del[2][2], cdel[2]{0,0};
    Vec3 rSP[2];

    for(int j=0; j<2; j++) {
      Vec3 rP1S2 = rS1S2 - rSP[0];
      int signj = j?-1:1;
      for(int k_=0; k_<z[j]; k_++) {
        double ep = k_*2*M_PI/z[j]+signi*ga[j];
        rSP[j](0) = -sin(ep);
        rSP[j](1) = cos(ep);
        rSP[j] = gearwheel[j]->getFrame()->getOrientation()*rSP[j];
        double cdel_ = signj*(rSP[j].T()*rP1S2/a);
        if(cdel_>cdel[j]) {
          cdel[j] = cdel_;
          k[i][j] = k_;
        }
      }
      rSP[j](0) = -sin(k[i][j]*2*M_PI/z[j]);
      rSP[j](1) = cos(k[i][j]*2*M_PI/z[j]);
      rSP[j] = gearwheel[j]->getFrame()->getOrientation()*rSP[j];
      cdel[j] = signj*(rSP[j].T()*rS1S2/a);
      del[i][j] = signi*signj*(gearwheel[j]->getFrame()->getOrientation().col(2).T()*crossProduct(rS1S2,rSP[j])>=0?acos(cdel[j]):-acos(cdel[j]));
      be[i][j] = ga[j] + del[i][j] + al;
      rSP[j](0) = signi*rb[j]*(sin(be[i][j])-cos(be[i][j])*be[i][j]);
      rSP[j](1) = rb[j]*(cos(be[i][j])+sin(be[i][j])*be[i][j]);
      rSP[j] = gearwheel[j]->getFrame()->getOrientation()*BasicRotAIKz(signi*ga[j]+k[i][j]*2*M_PI/z[j])*rSP[j];
      contact.getContourFrame(igearwheel[j])->setPosition(gearwheel[j]->getFrame()->getPosition() + rSP[j]);
    }

    contact.getGeneralizedRelativePosition(false)(0) = n1.T()*(contact.getContourFrame(igearwheel[1])->getPosition(false) - contact.getContourFrame(igearwheel[0])->getPosition(false));
  }

  void ContactKinematicsGearWheelGearWheel::updatewb(SingleContact &contact, int i) {
    int signi = i?-1:1;
    const Vec3 n1 = contact.getContourFrame(igearwheel[0])->evalOrientation().col(0);
//    const Vec3 vC1 = contact.getContourFrame(igearwheel[0])->evalVelocity();
//    const Vec3 vC2 = contact.getContourFrame(igearwheel[1])->evalVelocity();
    Vec3 rS1S2 = gearwheel[1]->getFrame()->evalPosition() - gearwheel[0]->getFrame()->evalPosition();
    Vec3 vS1S2 = gearwheel[1]->getFrame()->evalVelocity() - gearwheel[0]->getFrame()->evalVelocity();
    double ad = (vS1S2.T()*rS1S2)/a;
    double ald = a0*cos(al0)*ad/(a*a*sin(al));
    double deld[2], bed[2];

    Vec3 vrel[2];
    for(int j=0; j<2; j++) {
      deld[j] = signi*(gearwheel[j]->getFrame()->getAngularVelocity().T()*gearwheel[j]->getFrame()->getOrientation().col(2));
      bed[j] = deld[j] + ald;
      vrel[j](0) = signi*rb[j]*(cos(be[i][j])*bed[j] + sin(be[i][j])*bed[j]*be[i][j] - cos(be[i][j])*bed[j]);
      vrel[j](1) = rb[j]*(-sin(be[i][j])*bed[j] + cos(be[i][j])*bed[j]*be[i][j] + sin(be[i][j])*bed[j]);
      vrel[j] = gearwheel[j]->getFrame()->getOrientation()*BasicRotAIKz(signi*ga[j]+k[i][j]*2*M_PI/z[j])*vrel[j];
    }
    contact.getwb(false)(0) += /*parnPart1.T()*(vC2-vC1)+*/n1.T()*(crossProduct(contact.getContourFrame(igearwheel[1])->evalAngularVelocity(),vrel[1])-crossProduct(contact.getContourFrame(igearwheel[0])->evalAngularVelocity(),vrel[0]));
//    if (contact.getwb(false).size()>1)
//      contact.getwb(false)(1) +=
  }

}
