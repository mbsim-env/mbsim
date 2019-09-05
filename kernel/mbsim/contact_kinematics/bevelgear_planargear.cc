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
    m = bevelgear->getModule();
    al0 = bevelgear->getPressureAngle();
    double phi0 = tan(al0)-al0;
    double s0 = m*M_PI/2;
    double dk[2];
    z[0] = bevelgear->getNumberOfTeeth();
    beta[0] = bevelgear->getHelixAngle();
    beta[1] = planargear->getHelixAngle();
    d0[0] = m*z[0]/cos(beta[0]);
    db[0] = d0[0]*cos(al0);
    dk[0] = d0[0]+2*m;
    rb[0] = db[0]/2;
    sb[0] = db[0]*(s0/d0[0]+phi0)-bevelgear->getBacklash();
    ga[0] = sb[0]/rb[0]/2;
    delmin[0] = -al0-ga[0];
    delmax[0] = tan(acos(db[0]/dk[0]))-al0-ga[0];
    delmin[1] = -m/cos(al);
    delmax[1] = m/cos(al);
    z[1] = planargear->getNumberOfTeeth();
    sb[1] = m*(M_PI/2 + 2*tan(al0)) - planargear->getBacklash();
    d0[1] = m*z[1]/cos(beta[1]);
  }

  void ContactKinematicsBevelGearPlanarGear::updateg(SingleContact &contact, int ii) {
    contact.getGeneralizedRelativePosition(false)(0) = 1e10;
    al = al0;
    Vec3 ey1 = bevelgear->getFrame()->evalOrientation().T()*planargear->getFrame()->evalOrientation().col(1);
    Vec3 ez2 = planargear->getFrame()->getOrientation().T()*bevelgear->getFrame()->getOrientation().col(2);
    double phi1 = (ey1(0)>=0?1:-1)*acos(ey1(1)/sqrt(pow(ey1(0),2)+pow(ey1(1),2))); 
    double phi2 = (ez2(0)>=0?-1:1)*acos(ez2(2)/sqrt(pow(ez2(0),2)+pow(ez2(2),2))); 
    double d = sqrt(pow(d0[1],2)-pow(d0[0],2))/2;
    double be = asin(d0[0]/d0[1]);
    double delh2 = (M_PI/2-planargear->getBacklash()/m)/z[1];
    double delh1 = (M_PI/2-bevelgear->getBacklash()/m)/z[0];
    for(int i=0; i<2; i++) {
      int signi = i?-1:1;
      Vec3 rOP[2], rSP[2];
      vector<int> v[2];
      if(maxNumContacts==1) {
        int k = 0;
        double rsi = 1e10;
        int signk = (phi2 - signi*delh2)>0?-1:1;
        for(int k_=0; k_<z[1]; k_++) {
          double rsi_ = fabs(signk*k_*2*M_PI/z[1] - signi*delh2 + phi2);
          if(rsi_<rsi) {
            rsi = rsi_;
            k = k_;
          }
        }
        v[1].push_back(signk*k);
        double phi1corr = ((signk*k*2*M_PI/z[1] - signi*delh2 + phi2)*z[1])/z[0];

        k = 0;
        rsi = 1e10;
        signk = (phi1 + signi*delh1 - phi1corr)>0?-1:1;
        for(int k_=0; k_<z[0]; k_++) {
          double rsi_ = fabs(signk*k_*2*M_PI/z[0] + signi*delh1 + phi1 - phi1corr);
          if(rsi_<rsi) {
            rsi = rsi_;
            k = k_;
          }
        }
        v[0].push_back(signk*k);
      }
      else {
        throw runtime_error("The maximum number of contacts must be 1 at present");
      }

      double k[2];
      for (auto & i0 : v[0]) {
        for (auto & i1 : v[1]) {
          k[0] = i0;
          k[1] = i1;
          if(ii==0 or not(k[0]==ksave[0][0] and k[1]==ksave[0][1])) {
            double phi2q = -signi*delh2+k[1]*2*M_PI/z[1]+phi2;
            double l = -signi*sin(phi2q)/cos(phi2q-beta[1])*sin(al)*d0[1]/2;
            rSP[1](0) = signi*l*sin(al)*cos(beta[1]);
            rSP[1](1) = l*cos(al);
            rSP[1](2) = signi*l*sin(al)*sin(beta[1])+d0[1]/2;
            rSP[1] = planargear->getFrame()->getOrientation()*BasicRotAIKy(-signi*delh2+k[1]*2*M_PI/z[1])*rSP[1];
            rOP[1] = planargear->getFrame()->getPosition() + rSP[1];

            double phi1q = signi*delh1+k[0]*2*M_PI/z[0]+phi1;
            phi2q = -d0[0]/d0[1]*phi1q;
            l = signi*sin(phi2q)/cos(phi2q-beta[0])*sin(al)*d0[1]/2;
            double a = -signi*l*sin(al)*cos(phi2q-beta[0])+d0[1]/2*sin(phi2q);
            double b = l*cos(al)-d*sin(be);
            double c = signi*l*sin(al)*sin(phi2q-beta[0])+d0[1]/2*cos(phi2q)-d*cos(be);
            rSP[0](0) = -a*cos(phi1q)+b*sin(phi1q)*cos(be)-c*sin(phi1q)*sin(be);
            rSP[0](1) = a*sin(phi1q)+b*cos(phi1q)*cos(be)-c*cos(phi1q)*sin(be);
            rSP[0](2) = b*sin(be)+c*cos(be);
            rSP[0] = bevelgear->getFrame()->getOrientation()*BasicRotAIKz(signi*delh1+k[0]*2*M_PI/z[0])*rSP[0];
            rOP[0] = bevelgear->getFrame()->getPosition() + rSP[0];

            Vec3 u2(NONINIT);
            u2(0) = signi*sin(al)*cos(beta[1]);
            u2(1) = cos(al);
            u2(2) = signi*sin(al)*sin(beta[1]);
            u2 = planargear->getFrame()->getOrientation()*BasicRotAIKy(-signi*delh2+k[1]*2*M_PI/z[1])*u2;

            Vec3 v2(NONINIT);
            v2(0) = signi*sin(beta[1]);
            v2(1) = 0;
            v2(2) = -signi*cos(beta[1]);
            v2 = planargear->getFrame()->getOrientation()*BasicRotAIKy(-signi*delh2+k[1]*2*M_PI/z[1])*v2;

            Vec3 n2 = crossProduct(u2,v2);

            double g = n2.T()*(rOP[0]-rOP[1]);
            if(/**g>-0.5*M_PI*d0[0]/z[0] and **/g<contact.getGeneralizedRelativePosition(false)(0)) {
              ksave[ii][0] = k[0];
              ksave[ii][1] = k[1];
              etasave[ii][0] = -delh1-signi*(k[0]*2*M_PI/z[0]+phi1);
              etasave[ii][1] = delh2-signi*(k[1]*2*M_PI/z[1]+phi2);
              signisave[ii] = signi;

              contact.getContourFrame(iplanargear)->setPosition(rOP[1]);
              contact.getContourFrame(iplanargear)->getOrientation(false).set(0,n2);
              contact.getContourFrame(iplanargear)->getOrientation(false).set(1,u2);
              contact.getContourFrame(iplanargear)->getOrientation(false).set(2,v2);

              contact.getContourFrame(ibevelgear)->setPosition(rOP[0]);
              contact.getContourFrame(ibevelgear)->getOrientation(false).set(0, -contact.getContourFrame(iplanargear)->getOrientation(false).col(0));
              contact.getContourFrame(ibevelgear)->getOrientation(false).set(1, -contact.getContourFrame(iplanargear)->getOrientation(false).col(1));
              contact.getContourFrame(ibevelgear)->getOrientation(false).set(2, contact.getContourFrame(iplanargear)->getOrientation(false).col(2));

              contact.getGeneralizedRelativePosition(false)(0) = g;
            }
          }
        }
      }
    }
  }

  void ContactKinematicsBevelGearPlanarGear::updatewb(SingleContact &contact, int ii) {
    throw runtime_error("ContactKinematicsBevelGearPlanarGear::updatewb not yet implemented!");
  }

}
