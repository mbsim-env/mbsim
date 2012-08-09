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
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "sphere_frustum.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsSphereFrustum::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0; ifrustum = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      frustum = static_cast<Frustum*>(contour[1]);
    } 
    else {
      isphere = 1; ifrustum = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      frustum = static_cast<Frustum*>(contour[0]);
    }
  }


  void ContactKinematicsSphereFrustum::updateg(Vec &g, ContourPointData *cpData, int index) {

    // Bezugspunkt Kugel: Mittelpunkt
    // Bezugspunkt Kegel: Mittelpunkt Grundfläche
    // Rotationsachse Kegel: y-Achse
    Vec Wd = sphere->getFrame()->getPosition() - frustum->getFrame()->getPosition(); // Vektor von Bezugspunkt Kegel zu Bezugspunkt Kreis
    
    SqrMat Mat0 = frustum->getFrame()->getOrientation();
    Vec yAchse = Mat0.col(1);
    double loc = yAchse.T()*Wd; // Projektion Distanzvektor auf y-Achse
    Vec xAchse = Wd - (yAchse * loc);
    double l=nrm2(xAchse);
    SqrMat AW1(3);
    xAchse =  xAchse/l;
    Vec zAchse = crossProduct(xAchse,yAchse);
    AW1.col(0) = xAchse;
    AW1.col(1) = yAchse;
    AW1.col(2) = zAchse;
      
    //int fall = 0;
    double h = frustum->getHeight();
    Vec r = frustum->getRadii(); // r(0): Basisradius, r(1): Topradius
    double r_h = r(0) + (r(1)-r(0))/h * loc; // Radius an der Stelle des Kreismittelpunkts
    

    if(loc<-sphere->getRadius() || loc>h+sphere->getRadius() || fabs(l)<epsroot()) { // TODO! rudimentäre Bestimmung ob Kontakt
      g(0) = 1;
    }
    else {
      // Halber Oeffnungswinkel
      double  psi = atan((r(0) - r(1))/h); // psi > 0 falls r_unten(0) > r_oben(1)
      double phi = M_PI*0.5 - fabs(psi);
      double vz=psi/fabs(psi);
      g(0) = fabs(r_h-l)*sin(phi)-sphere->getRadius();
      int out = 1;

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
        cpData[ifrustum].getFrameOfReference().getPosition() = frustum->getFrame()->getPosition() + r(0)*xAchse + loc*yAchse;
        cpData[ifrustum].getFrameOfReference().getOrientation().col(0) = out*xAchse;
        cpData[ifrustum].getFrameOfReference().getOrientation().col(1) = out*yAchse;
        cpData[ifrustum].getFrameOfReference().getOrientation().col(2) = crossProduct(xAchse,yAchse);
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
        SqrMat A(3); // Drehmatrix
        A(2,2) = 1;
        A(0,0) = cos(alpha);
        A(1,1) = cos(alpha);
        A(0,1) = -sin(alpha);
        A(1,0) = sin(alpha);

        double v1 = loc/sin(phi);
        double v2 = vz*cos(phi)*(r_h-l);
        double v12 = out*(v1+v2);
     
        // System of frustum
        cpData[ifrustum].getFrameOfReference().getPosition() = frustum->getFrame()->getPosition() + (r(0)*xAchse);
        cpData[ifrustum].getFrameOfReference().getOrientation() = AW1 * A;
        cpData[ifrustum].getFrameOfReference().getPosition() = cpData[ifrustum].getFrameOfReference().getPosition() + v12*cpData[ifrustum].getFrameOfReference().getOrientation().col(1);
      }

      // System of sphere (position)
      cpData[isphere].getFrameOfReference().getPosition() = cpData[ifrustum].getFrameOfReference().getPosition() + g(0)*cpData[ifrustum].getFrameOfReference().getOrientation().col(0);
      // System of sphere (orientation)
      cpData[isphere].getFrameOfReference().getOrientation().col(0) = -cpData[ifrustum].getFrameOfReference().getOrientation().col(0);
      cpData[isphere].getFrameOfReference().getOrientation().col(1) = -cpData[ifrustum].getFrameOfReference().getOrientation().col(1);
      cpData[isphere].getFrameOfReference().getOrientation().col(2) = crossProduct(cpData[isphere].getFrameOfReference().getOrientation().col(0),cpData[isphere].getFrameOfReference().getOrientation().col(1));

    }
  }
}

