/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */

#include <config.h> 
#include "mbsimFlexibleBody/contact_kinematics/circlehollow_cylinderflexible.h"
#include "mbsimFlexibleBody/contours/cylinder_flexible.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/functions_contact.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

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

    Vec WrOP_circle =  circle->getFrame()->getPosition();

    // longitudinal contact search
    Contact1sSearch search(func);
    search.setNodes(cylinder->getNodes());

    if(cpData[icylinder].getLagrangeParameterPosition().size()==2 ) {
      search.setInitialValue(cpData[icylinder].getLagrangeParameterPosition()(0));
    } 
    else {
      search.setSearchAll(true);
      cpData[icylinder].getLagrangeParameterPosition() = Vec(2);
    }

    cpData[icylinder].getLagrangeParameterPosition()(0) = search.slv();

    // kinematics for azimuthal contact search
    cylinder->updateKinematicsForFrame(cpData[icylinder],position_cosy);
    Vec WrD = cpData[icylinder].getFrameOfReference().getPosition() - WrOP_circle;

    const double &R = circle->getRadius();
    const double &r = cylinder->getRadius();

    Vec WbK = circle->getReferenceOrientation().col(2);
    Vec WtB = cpData[icylinder].getFrameOfReference().getOrientation().col(1); // normal in first column

    double cos_alpha = WtB.T() * WbK;

    if( nrm2(WrD) > 0 ) { // conic section theory
      double a = abs(r/cos_alpha);

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
      cpData[icylinder].getLagrangeParameterPosition()(1) = searchPhi.slv(); 
      Vec dTilde = funcPhi->computeWrD(cpData[icylinder].getLagrangeParameterPosition()(1));

      delete funcPhi;

      cpData[icircle].getFrameOfReference().getPosition() = WrOP_circle + R * dTilde/nrm2(dTilde);
      cylinder->updateKinematicsForFrame(cpData[icylinder],position_cosy);
      Vec WrD2 = cpData[icylinder].getFrameOfReference().getPosition() - cpData[icircle].getFrameOfReference().getPosition() ;

      Vec3 normal = (WrD - WtB.T()*WrD*WtB );

      g(0) = nrm2(WrD2);
      if( nrm2(normal)>0.01*a ) { // hack of Roland Zander, oh my god; what about the COSY in the non-if-case? TODO
        cpData[icircle].getFrameOfReference().getOrientation().set(0, normal/nrm2(normal));
        cpData[icylinder].getFrameOfReference().getOrientation().set(0, - cpData[icircle].getFrameOfReference().getOrientation().col(0));
        cpData[icylinder].getFrameOfReference().getOrientation().set(2, crossProduct(cpData[icylinder].getFrameOfReference().getOrientation().col(0),cpData[icylinder].getFrameOfReference().getOrientation().col(1)));

        g(0) = cpData[icylinder].getFrameOfReference().getOrientation().col(0).T() * WrD2 ;
      }
    }
    else { // only dimensioning
      cpData[icircle].getFrameOfReference().setPosition(WrOP_circle);
      cpData[icircle].getFrameOfReference().getOrientation().set(0, -cpData[icylinder].getFrameOfReference().getOrientation().col(0));
      cpData[icircle].getFrameOfReference().getOrientation().set(1, -cpData[icylinder].getFrameOfReference().getOrientation().col(1));
      cpData[icircle].getFrameOfReference().getOrientation().set(2, cpData[icylinder].getFrameOfReference().getOrientation().col(2));

      g(0) = R*abs(cos_alpha) - r ;
    }
  }

}

