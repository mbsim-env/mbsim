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
#include "sphere_frustum.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsSphereFrustum::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
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

  void ContactKinematicsSphereFrustum::updateg(SingleContact &contact, int i) {

    // Bezugspunkt Kugel: Mittelpunkt
    // Bezugspunkt Kegel: Mittelpunkt Grundfläche
    // Rotationsachse Kegel: y-Achse
    Vec3 Wd = sphere->getFrame()->evalPosition() - frustum->getFrame()->evalPosition(); // Vektor von Bezugspunkt Kegel zu Bezugspunkt Kreis
    
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
    
    double g;
    if(loc<-sphere->getRadius() || loc>h+sphere->getRadius() || fabs(l)<epsroot) { // TODO! rudimentäre Bestimmung ob Kontakt
      g = 1;
    }
    else {
      // Halber Oeffnungswinkel
      double  psi = atan((r(0) - r(1))/h); // psi > 0 falls r_unten(0) > r_oben(1)
      double phi = M_PI*0.5 - fabs(psi);
      double vz=sign(psi);
      g = fabs(r_h-l)*sin(phi)-sphere->getRadius();
      double out = 1;

      // Fallabfrage
      if(fabs(psi)<epsroot) {
        // Zylinder
        if(l-r(0) > 0) {
          //fall = 1; // außen
        }
        else {
          //fall = 2; // innen
          out = -1;
        }
        
        // System of frustum
        contact.getContourFrame(ifrustum)->setPosition(frustum->getFrame()->getPosition() + r(0)*xAchse + loc*yAchse);
        contact.getContourFrame(ifrustum)->getOrientation(false).set(0, out*xAchse);
        contact.getContourFrame(ifrustum)->getOrientation(false).set(1, out*yAchse);
        contact.getContourFrame(ifrustum)->getOrientation(false).set(2, crossProduct(xAchse,yAchse));
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
        contact.getContourFrame(ifrustum)->setPosition(frustum->getFrame()->getPosition() + (r(0)*xAchse));
        contact.getContourFrame(ifrustum)->setOrientation(AW1 * A);
        contact.getContourFrame(ifrustum)->setPosition(contact.getContourFrame(ifrustum)->getPosition(false) + v12*contact.getContourFrame(ifrustum)->getOrientation(false).col(1));
      }

      // System of sphere (position)
      contact.getContourFrame(isphere)->setPosition(contact.getContourFrame(ifrustum)->getPosition(false) + g*contact.getContourFrame(ifrustum)->getOrientation(false).col(0));
      // System of sphere (orientation)
      contact.getContourFrame(isphere)->getOrientation(false).set(0, -contact.getContourFrame(ifrustum)->getOrientation(false).col(0));
      contact.getContourFrame(isphere)->getOrientation(false).set(1, -contact.getContourFrame(ifrustum)->getOrientation(false).col(1));
      contact.getContourFrame(isphere)->getOrientation(false).set(2, crossProduct(contact.getContourFrame(isphere)->getOrientation(false).col(0),contact.getContourFrame(isphere)->getOrientation(false).col(1)));

    }
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }
}
