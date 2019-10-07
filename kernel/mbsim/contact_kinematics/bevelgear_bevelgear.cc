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
#include "bevelgear_bevelgear.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/bevel_gear.h"
#include <mbsim/utils/rotarymatrices.h>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsBevelGearBevelGear::assignContours(const vector<Contour*> &contour) {
    gear[0] = static_cast<BevelGear*>(contour[0]);
    gear[1] = static_cast<BevelGear*>(contour[1]);
    igear[0] = 0;
    igear[1] = 1;
    beta[0] = gear[0]->getHelixAngle();
    beta[1] = gear[1]->getHelixAngle();
    m = gear[0]->getModule()/cos(beta[0]);
    al0 = gear[0]->getPressureAngle();
    z[0] = gear[0]->getNumberOfTeeth();
    z[1] = gear[1]->getNumberOfTeeth();
    delh2 = (M_PI/2-gear[1]->getBacklash()/m)/z[1];
    delh1 = (M_PI/2-gear[0]->getBacklash()/m)/z[0];
  }

  void ContactKinematicsBevelGearBevelGear::updateg(SingleContact &contact, int ii) {
    contact.getGeneralizedRelativePosition(false)(0) = 1e10;
    Vec3 ez1 = gear[0]->getFrame()->evalOrientation().T()*(-gear[1]->getFrame()->evalOrientation().col(2));
    Vec3 ez2 = gear[1]->getFrame()->getOrientation().T()*(-gear[0]->getFrame()->getOrientation().col(2));
    double phi1 = (ez1(0)>=0?1:-1)*acos(ez1(1)/sqrt(pow(ez1(0),2)+pow(ez1(1),2))); 
    double phi2 = (ez2(0)>=0?1:-1)*acos(ez2(1)/sqrt(pow(ez2(0),2)+pow(ez2(1),2))); 
    if(nrm2(gear[1]->getFrame()->evalPosition()-gear[0]->getFrame()->evalPosition()+m/2*(z[0]/tan(gear[0]->getPitchAngle())*gear[0]->getFrame()->getOrientation().col(2)-z[1]/tan(gear[1]->getPitchAngle())*gear[1]->getFrame()->getOrientation().col(2)))>1e-8)
      msg(Warn)<<"Large devitation detected at t="<<gear[1]->getTime()<<"\nContact kinematics may be wrong!" <<endl;

    for(int i=0; i<2; i++) {
      int signi = i?-1:1;
      Vec3 rOP[2];
      vector<int> v[2];
      if(maxNumContacts==1) {
        int k = 0;
        double rsi = 1e10;
        int signk = (phi2 - signi*delh2)>0?-1:1;
        for(int k_=0; k_<z[1]; k_++) {
          double rsi_ = fabs(phi2 + signk*k_*2*M_PI/z[1] - signi*delh2);
          if(rsi_<rsi) {
            rsi = rsi_;
            k = k_;
          }
        }
        v[1].push_back(signk*k);
        double phi1corr = ((phi2 + signk*k*2*M_PI/z[1] - signi*delh2)*z[1])/z[0];

        k = 0;
        rsi = 1e10;
        signk = (phi1 - signi*delh1 + phi1corr)>0?-1:1;
        for(int k_=0; k_<z[0]; k_++) {
          double rsi_ = fabs(phi1 + signk*k_*2*M_PI/z[0] - signi*delh1 + phi1corr);
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
            Vec2 zeta1(NONINIT);
            zeta1(0) = -(phi1+k[0]*2*M_PI/z[0]-signi*delh1);
            double phi2q = -sin(gear[0]->getPitchAngle())*zeta1(0);
            double a = pow(cos(phi2q-beta[0]),2)+pow(sin(al0)*sin(phi2q-beta[0]),2);
            double b = m*z[0]/sin(gear[0]->getPitchAngle())*(cos(beta[0])*pow(cos(phi2q-beta[0]),2)+pow(sin(al0),2)*sin(phi2q)*sin(phi2q-beta[0])-pow(sin(al0),2)*sin(beta[0])*cos(phi2q-beta[0])*sin(phi2q-beta[0]));
            double c = pow(m*z[0]/sin(gear[0]->getPitchAngle())/2*sin(al0),2)*(pow(sin(phi2q),2)-2*sin(beta[0])*cos(phi2q-beta[0])*sin(phi2q));
            zeta1(1) = (-b+sqrt(b*b-4*a*c))/2/a;
            gear[0]->setFlank(signi);
            gear[0]->setTooth(k[0]);
            rOP[0] = gear[0]->evalPosition(zeta1);

            Vec2 zeta2(NONINIT);
            zeta2(0) = -(phi2+k[1]*2*M_PI/z[1]-signi*delh1);
            phi2q = -sin(gear[1]->getPitchAngle())*zeta2(0);
            a = pow(cos(phi2q-beta[1]),2)+pow(sin(al0)*sin(phi2q-beta[1]),2);
            b = m*z[1]/sin(gear[1]->getPitchAngle())*(cos(beta[1])*pow(cos(phi2q-beta[1]),2)+pow(sin(al0),2)*sin(phi2q)*sin(phi2q-beta[1])-pow(sin(al0),2)*sin(beta[1])*cos(phi2q-beta[1])*sin(phi2q-beta[1]));
            c = pow(m*z[1]/sin(gear[1]->getPitchAngle())/2*sin(al0),2)*(pow(sin(phi2q),2)-2*sin(beta[1])*cos(phi2q-beta[1])*sin(phi2q));
            zeta2(1) = (-b+sqrt(b*b-4*a*c))/2/a;
            gear[1]->setFlank(signi);
            gear[1]->setTooth(k[1]);
            rOP[1] = gear[1]->evalPosition(zeta2);

            Vec n1 = gear[0]->evalWn(zeta1);

            double g = n1.T()*(rOP[1]-rOP[0]);
            if(g>-0.5*M_PI*m and g<contact.getGeneralizedRelativePosition(false)(0)) {
              ksave[ii][0] = k[0];
              ksave[ii][1] = k[1];
              signisave[ii] = signi;

              contact.getContourFrame(igear[0])->setZeta(zeta1);
              contact.getContourFrame(igear[0])->setPosition(rOP[0]);
              contact.getContourFrame(igear[0])->getOrientation(false).set(0,n1);
              contact.getContourFrame(igear[0])->getOrientation(false).set(1,gear[0]->evalWu(zeta1));
              contact.getContourFrame(igear[0])->getOrientation(false).set(2,crossProduct(contact.getContourFrame(igear[0])->getOrientation(false).col(0),contact.getContourFrame(igear[0])->getOrientation(false).col(1)));

              contact.getContourFrame(igear[1])->setZeta(zeta2);
              contact.getContourFrame(igear[1])->setPosition(rOP[1]);
              contact.getContourFrame(igear[1])->getOrientation(false).set(0,gear[1]->evalWn(zeta1));
              contact.getContourFrame(igear[1])->getOrientation(false).set(1,gear[1]->evalWu(zeta2));
              contact.getContourFrame(igear[1])->getOrientation(false).set(2,crossProduct(contact.getContourFrame(igear[1])->getOrientation(false).col(0),contact.getContourFrame(igear[1])->getOrientation(false).col(1)));

              contact.getGeneralizedRelativePosition(false)(0) = g;
            }
          }
        }
      }
    }
  }

  void ContactKinematicsBevelGearBevelGear::updatewb(SingleContact &contact, int ii) {
    throw runtime_error("ContactKinematicsBevelGearBevelGear::updatewb not yet implemented!");
  }

}
