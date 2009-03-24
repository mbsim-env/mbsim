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
 * Contact: rzander@users.berlios.de
 */

#include <config.h> 
#include <mbsim/contact_kinematics/circlehollow_cylinderflexible.h>
#include <mbsim/contour.h>
#include <mbsim/functions_contact.h>

namespace MBSim {

  ContactKinematicsCircleHollowCylinderFlexible::~ContactKinematicsCircleHollowCylinderFlexible() {
    delete func;
  }

  void ContactKinematicsCircleHollowCylinderFlexible::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<CircleHollow*>(contour[0])) {
      icircle = 0;
      icylinder = 1;
      circle = static_cast<CircleHollow*>(contour[0]);
      cylinder = static_cast<CylinderFlexible*>(contour[1]);
    } 
    else {
      icircle = 1;
      icylinder = 0;
      circle = static_cast<CircleHollow*>(contour[1]);
      cylinder = static_cast<CylinderFlexible*>(contour[0]);
    }
    func= new FuncPairContour1sCircleHollow(circle,cylinder);
  }

  void ContactKinematicsCircleHollowCylinderFlexible::updateg(Vec &g, ContourPointData *cpData) {

    Vec WrOP_circle =  circle->getWrOP();

    // longitudinal contact search
    Contact1sSearch search(func);
    search.setNodes(cylinder->getNodes());

    if(cpData[icylinder].alpha.size()==2 ) {
      search.setInitialValue(cpData[icylinder].alpha(0));
    } 
    else {
      search.setSearchAll(true);
      cpData[icylinder].alpha = Vec(2);
    }

    cpData[icylinder].alpha(0) = search.slv();

    // kinematics for azimuthal contact search
    cylinder->updateKinematicsForFrame( cpData[icylinder] );
    Vec WrD = cpData[icylinder].cosy.getPosition() - WrOP_circle;

    const double &R = circle->getRadius();
    const double &r = cylinder->getRadius();

    Vec WbK = circle->computeWb();
    Vec WtB = cpData[icylinder].cosy.getOrientation().col(1); // normal in first column

    double cos_alpha = trans( WtB ) * WbK;

    if( nrm2(WrD) > 0 ) { // conic section theory
      double a   =  abs(r/cos_alpha);

      Vec be1, be2;
      if( -0.99750 < cos_alpha && cos_alpha < 0.99750) { // until 1 degree tilting
        be2 = crossProduct(WtB,WbK); be2 /= nrm2(be2);
        be1 = crossProduct(be2,WbK); be1 /= nrm2(be1);
      }
      else {
        if(nrm2(WrD) > 0.01*a) {
          be2 = WrD/nrm2(WrD);
        }
        else {
          // compute arbitrary normal to cylinder tangent
          be2 = Vec(3,NONINIT); be2(0) = WtB(1); be2(1) = - WtB(0); be2(2) = 0.0;
          be2 /= nrm2(be2);
        }
        be1 = crossProduct(WbK,be2); be1 /=nrm2(be1);// b1;
      }

      FuncPairEllipseCircle *funcPhi= new FuncPairEllipseCircle(R, a, r);

      funcPhi->setDiffVec(WrD);
      funcPhi->setEllipseCOS(be1,be2);

      Contact1sSearch searchPhi(funcPhi);
      searchPhi.setSearchAll(true);
      static const int                SEC =   16;
      static const double            dphi =   2*M_PI/SEC * 1.02;      // enlargement for better contact convergence
      static const double phiStartSpacing = - 2*M_PI/SEC * 0.02 / 2.; 
      searchPhi.setEqualSpacing(SEC,phiStartSpacing,dphi);
      cpData[icylinder].alpha(1) = searchPhi.slv(); 
      Vec dTilde = funcPhi->computeWrD(cpData[icylinder].alpha(1));

      delete funcPhi;

      cpData[icircle].WrOC = WrOP_circle + R * dTilde/nrm2(dTilde);
      cylinder->updateKinematicsForFrame( cpData[icylinder] );
      Vec WrD2 = cpData[icylinder].cosy.getPosition() - cpData[icircle].WrOC ;

      Vec normal = (WrD - trans(WtB)*WrD*WtB );

      g(0) = nrm2(WrD2);
      if( nrm2(normal)>0.01*a ) { // hack of Roland Zander, oh my god; what about the COSY in the non-if-case? TODO
        cpData[icircle].cosy.getOrientation().col(0) = normal/nrm2(normal);
        cpData[icylinder].cosy.getOrientation().col(0) = - cpData[icircle].Wn;
        cpData[icylinder].cosy.getOrientation().col(2) = crossProduct(cpData[icylinder].cosy.getOrientation().col(0),cpData[icylinder].cosy.getOrientation().col(1));

        g(0) = trans(cpData[icylinder].Wn) * WrD2 ;
      }
    }
    else { // only dimensioning
      cpData[icircle].cosy.setPosition(WrOP_circle);
      cpData[icircle].cosy.getOrientation().col(0) = -cpData[icylinder].cosy.getOrientation().col(0);
      cpData[icircle].cosy.getOrientation().col(1) = -cpData[icylinder].cosy.getOrientation().col(1);
      cpData[icircle].cosy.getOrientation().col(2) = cpData[icylinder].cosy.getOrientation().col(2);

      g(0) = R*abs(cos_alpha) - r ;
    }
  }

}

