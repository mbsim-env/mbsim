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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "mbsim/contact_kinematics/circlesolid_frustum2d.h"
#include "mbsim/contour.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  void ContactKinematicsCircleSolidFrustum2D::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleSolid*>(contour[0])) {
      icircle = 0; ifrustum = 1;
      circle = static_cast<CircleSolid*>(contour[0]);
      frustum = static_cast<Frustum2D*>(contour[1]);
    } 
    else {
      icircle = 1; ifrustum = 0;
      circle = static_cast<CircleSolid*>(contour[1]);
      frustum = static_cast<Frustum2D*>(contour[0]);
    }
  }

  void ContactKinematicsCircleSolidFrustum2D::updateg(Vec &g, ContourPointData *cpData) {

    // Bezugspunkt Kreis: Mittelpunkt
    // Bezugspunkt Kegel: Mittelpunkt Grundfläche
    // Rotationsachse Kegel: y-Achse
    // z-Achse Kegel / Kreis: aus Zeichenebene heraus
    Vec Wd = circle->getFrame()->getPosition() - frustum->getFrame()->getPosition(); // Vektor von Bezugspunkt Kegel zu Bezugspunkt Kreis
    cpData[ifrustum].getFrameOfReference().getOrientation() = frustum->getFrame()->getOrientation(); // Bezugssystem Kegel und Kontaktpunktsystem Kegel deckungsgleich
    cpData[ifrustum].getFrameOfReference().getPosition() = frustum->getFrame()->getPosition();
    double l0 = trans(Wd)*cpData[ifrustum].getFrameOfReference().getOrientation().col(0); // Projektion Distanzvektor auf x-Achse
    double l = l0;
    int fall = 0;
    if(l0<0) { // Kreis links von Kegelachse
      l *=-1;
    }
    double loc = trans(Wd)*cpData[ifrustum].getFrameOfReference().getOrientation().col(1); // Projektion Distanzvektor auf y-Achse
    double h = frustum->getHeight();
    Vec r = frustum->getRadii(); // r(0): Basisradius, r(1): Topradius
    double r_h = r(0) + (r(1)-r(0))/h * loc; // Radius an der Stelle des Kreismittelpunkts

    if(loc<0 || loc>h) { // TODO! rudimentäre Bestimmung ob Kontakt
      //active = false;
      g(0) = 1;
    }
    else {
      // Halber Oeffnungswinkel
      double  psi = atan((r(0) - r(1))/h); // psi > 0 falls r(0)nten(0) > r(1)ben(1)
      if(psi==0) {
        throw new MBSimError("ERROR (Cylinder): Not implemented!");
      }
      double phi = M_PI*0.5 - fabs(psi);
      double alpha=0; // Winkel der Drehmatrix

      if(l-r_h > -epsroot() && l0>0)
      {
        alpha=-psi; // außen rechts
        fall = 1;
      }
      if(l-r_h > -epsroot() && l0<0) {
        alpha = psi+M_PI; // außen links
        fall = 2;
      }
      if(l-r_h < epsroot() && l0>0) {
        alpha=-M_PI - psi; // innen rechts
        fall = 3;
      }
      if(l-r_h < epsroot() && l0<0) {
        alpha = psi; // innen links
        fall = 4;
      }

      SqrMat A(3); // Drehmatrix
      A(2,2) = 1;
      A(0,0) = cos(alpha);
      A(1,1) = cos(alpha);
      A(0,1) = sin(alpha);
      A(1,0) = -sin(alpha);
      

      // Circle outside frustum
      if(fall == 1 || fall == 2) {
        g(0) = (l-r_h)*sin(phi)-circle->getRadius(); // Berechnung Normalabstand
        double dh = cos(phi)*(circle->getRadius()+g(0));
        double dy_F, dx_F; // Verschiebung Bezugssystem Kegel
        if(r(0)>=r(1))
          dy_F = -dh + loc;
        else
          dy_F = dh + loc;
        double dr = dh / tan(phi);
        if(r(0)>=r(1))
          dx_F = r_h+dr;
        else
          dx_F = r_h-dr;

        double dx_C = -circle->getRadius()*sin(phi); // Verschiebung Bezugssystem Kreis
        double dy_C = -circle->getRadius()*cos(phi);
        if(r(0)<r(1))
          dy_C = circle->getRadius()*cos(phi);

        Vec d_C(3), d_F(3);
        d_C(0) = l0/l*dx_C;
        d_C(1) = dy_C;
        d_F(0) = l0/l*dx_F;
        d_F(1) = dy_F;

        // System of frustum (position)
        cpData[ifrustum].getFrameOfReference().getPosition() = frustum->getFrame()->getPosition() + d_F;
        // System of circle (position)
        cpData[icircle].getFrameOfReference().getPosition()=circle->getFrame()->getPosition() + d_C;
        // System of frustum (orientation)
        cpData[ifrustum].getFrameOfReference().getOrientation() = A * cpData[ifrustum].getFrameOfReference().getOrientation();
        // System of circle (orientation)
        cpData[icircle].getFrameOfReference().getOrientation().col(0) = -cpData[ifrustum].getFrameOfReference().getOrientation().col(0);
        cpData[icircle].getFrameOfReference().getOrientation().col(1) = -cpData[ifrustum].getFrameOfReference().getOrientation().col(1);
      }
      
      // Circle inside frustum
      if(fall == 3 || fall == 4) {
        g(0) = (r_h-l)*sin(phi)-circle->getRadius();
        double dh = cos(phi)*(circle->getRadius()+g(0));
        double dy_F;
        if(r(0)>=r(1))
          dy_F = dh + loc;
        else
          dy_F = -dh + loc;
        double dr = sin(phi)*(circle->getRadius()+g(0));
        double dx_F = l+dr;
        double dx_C = -circle->getRadius()*sin(phi);
        if(l0<0)
          dx_C *= -1;
        double dy_C = circle->getRadius()*cos(phi);
        if(r(0)<r(1))
          dy_C = -circle->getRadius()*cos(phi);

        Vec d_C(3), d_F(3);
        d_C(0) = -dx_C;
        d_C(1) = dy_C;
        d_F(0) = dx_F;
        d_F(1) = dy_F;

        // System of frustum (position)
        cpData[ifrustum].getFrameOfReference().getPosition()=frustum->getFrame()->getPosition() + d_F;
        // System of circle (position)
        cpData[icircle].getFrameOfReference().getPosition() = circle->getFrame()->getPosition() + d_C;
        // System of frustum (orientation)
        cpData[ifrustum].getFrameOfReference().getOrientation() = A * cpData[ifrustum].getFrameOfReference().getOrientation();
        // System of circle (orientation)
        cpData[icircle].getFrameOfReference().getOrientation().col(0) = -cpData[ifrustum].getFrameOfReference().getOrientation().col(0);
        cpData[icircle].getFrameOfReference().getOrientation().col(1) = -cpData[ifrustum].getFrameOfReference().getOrientation().col(1);
      }
    }
  }
}
