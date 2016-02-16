/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/contact_kinematics/circle_planarfrustum.h"
#include "mbsim/frames/frame.h"
#include "mbsim/contours/planar_frustum.h"
#include "mbsim/contours/circle.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCirclePlanarFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0; ifrustum = 1;
      circle = static_cast<Circle*>(contour[0]);
      frustum = static_cast<PlanarFrustum*>(contour[1]);
    } 
    else {
      icircle = 1; ifrustum = 0;
      circle = static_cast<Circle*>(contour[1]);
      frustum = static_cast<PlanarFrustum*>(contour[0]);
    }
  }


  void ContactKinematicsCirclePlanarFrustum::updateg(double t, double &g, std::vector<Frame*> &cFrame, int index) {
    
    Vec3 Wd = circle->getFrame()->getPosition(t) - frustum->getFrame()->getPosition(t);
    SqrMat3 Mat0 = frustum->getFrame()->getOrientation();
    Vec3 yAchse = Mat0.col(1);
    double loc = yAchse.T()*Wd; // Projektion Distanzvektor auf y-Achse
    Vec3 xAchse = Wd - (yAchse * loc);
    double l=nrm2(xAchse);
    SqrMat3 AW1;
    xAchse =  xAchse/l;
    Vec3 zAchse = crossProduct(xAchse,yAchse);
    AW1.set(0, xAchse);
    AW1.set(1, yAchse);
    AW1.set(2, zAchse);

    //int fall = 0;
    double h = frustum->getHeight();
    Vec2 r = frustum->getRadii(); // r(0): Basisradius, r(1): Topradius
    double r_h = r(0) + (r(1)-r(0))/h * loc; // Radius an der Stelle des Kreismittelpunkts
    

    if(loc<-circle->getRadius() || loc>h+circle->getRadius() || fabs(l)<epsroot()) {
      g = 1;
    }
    else {
      // Halber Oeffnungswinkel
      double  psi = atan((r(0) - r(1))/h); // psi > 0 falls r_unten(0) > r_oben(1)
      double phi = M_PI*0.5 - fabs(psi);
      double vz=psi/fabs(psi);
      g = fabs(r_h-l)*sin(phi)-circle->getRadius();
      double out = 1;

      // Fallabfrage
      if(fabs(psi)<epsroot()) {
        // Zylinder
        if(l-r(0) > 0) {
          //fall = 1; // außen
        }
        else {
          //fall = 2; // innen
          out = -1;
        }
        
        // System of frustum
        cFrame[ifrustum]->setPosition(frustum->getFrame()->getPosition() + r(0)*xAchse + loc*yAchse);
        cFrame[ifrustum]->getOrientation(false).set(0, out*xAchse);
        cFrame[ifrustum]->getOrientation(false).set(1, out*yAchse);
        cFrame[ifrustum]->getOrientation(false).set(2, crossProduct(xAchse,yAchse));
      }


          else {
        // Kegel
        if(l-r_h > -1e-5)
        {
          //fall = 3; // außen
        }
        else {
          //fall = 4; // innen
          out = -1;
        }

        double alpha= vz*(out*M_PI/2.-phi);
        SqrMat3 A; // Drehmatrix
        A(2,2) = 1;
        A(0,0) = cos(alpha);
        A(1,1) = cos(alpha);
        A(0,1) = -sin(alpha);
        A(1,0) = sin(alpha);

        double v1 = loc/sin(phi);
        double v2 = vz*cos(phi)*(r_h-l);
        double v12 = out*(v1+v2);
     
        // System of frustum
        cFrame[ifrustum]->setPosition(frustum->getFrame()->getPosition() + (r(0)*xAchse));
        cFrame[ifrustum]->setOrientation(AW1 * A);
        cFrame[ifrustum]->setPosition(cFrame[ifrustum]->getPosition(false) + v12*cFrame[ifrustum]->getOrientation(false).col(1));
      }

      // System of circle (position)
      cFrame[icircle]->setPosition(cFrame[ifrustum]->getPosition(false) + g*cFrame[ifrustum]->getOrientation(false).col(0));
      // System of circle (orientation)
      cFrame[icircle]->getOrientation(false).set(0, -cFrame[ifrustum]->getOrientation(false).col(0));
      cFrame[icircle]->getOrientation(false).set(1, -cFrame[ifrustum]->getOrientation(false).col(1));
      cFrame[icircle]->getOrientation(false).set(2, cFrame[ifrustum]->getOrientation(false).col(2));
    }
  }
}
