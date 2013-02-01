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

#ifndef _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_
#define _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "mbsimFlexibleBody/contact_kinematics/circlehollow_cylinderflexible.h"
#include "mbsimFlexibleBody/contours/cylinder_flexible.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/functions_contact.h"

namespace MBSimFlexibleBody {

  template <class Col>
  class CylinderFlexible;

  /**
   * \brief pairing CircleHollow to CylinderFlexible
   * \author Roland Zander
   * \date 2009-07-28 pure virtual updates (Thorsten Schindler)
   * \todo change stage to new interface TODO
   */
  template <class Col>
  class ContactKinematicsCircleHollowCylinderFlexible : public MBSim::ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsCircleHollowCylinderFlexible() {
      }

      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsCircleHollowCylinderFlexible();

      /* INHERITED INTERFACE */
      virtual void assignContours(const std::vector<MBSim::Contour*> &contour);
      virtual void updateg(fmatvec::Vec &g, MBSim::ContourPointData *cpData);
      virtual void updatewb(fmatvec::Vec &wb, const fmatvec::Vec &g, MBSim::ContourPointData* cpData) {
        throw MBSim::MBSimError("ERROR (ContactKinematicsCircleHollowCylinderFlexible::updatewb): Not implemented!");
      }
      ;
      /***************************************************/

    private:
      /**
       * \brief contour index
       */
      int icircle, icylinder;

      /**
       * \brief contour classes
       */
      MBSim::CircleHollow *circle;
      CylinderFlexible<Col> *cylinder;

      /**
       * \brief root function
       */
      MBSim::FuncPairContour1sCircleHollow *func;
  };

  template <class Col>
  inline ContactKinematicsCircleHollowCylinderFlexible<Col>::~ContactKinematicsCircleHollowCylinderFlexible() {
    delete func;
  }

  template <class Col>
  inline void ContactKinematicsCircleHollowCylinderFlexible<Col>::assignContours(const std::vector<MBSim::Contour*> &contour) {
    if (dynamic_cast<MBSim::CircleHollow*>(contour[0])) {
      icircle = 0;
      icylinder = 1;
      circle = static_cast<MBSim::CircleHollow*>(contour[0]);
      cylinder = static_cast<CylinderFlexible<Col>*>(contour[1]);
    }
    else {
      icircle = 1;
      icylinder = 0;
      circle = static_cast<MBSim::CircleHollow*>(contour[1]);
      cylinder = static_cast<CylinderFlexible<Col>*>(contour[0]);
    }
    func = new MBSim::FuncPairContour1sCircleHollow(circle, cylinder);
  }

  template <class Col>
  inline void ContactKinematicsCircleHollowCylinderFlexible<Col>::updateg(fmatvec::Vec &g, MBSim::ContourPointData *cpData) {

    fmatvec::Vec3 WrOP_circle = circle->getFrame()->getPosition();

    // longitudinal contact search
    MBSim::Contact1sSearch search(func);
    search.setNodes(cylinder->getNodes());

    if (cpData[icylinder].getLagrangeParameterPosition().size() == 2) {
      search.setInitialValue(cpData[icylinder].getLagrangeParameterPosition()(0));
    }
    else {
      search.setSearchAll(true);
      cpData[icylinder].getLagrangeParameterPosition() = fmatvec::Vec2();
    }

    cpData[icylinder].getLagrangeParameterPosition()(0) = search.slv();

    // kinematics for azimuthal contact search
    cylinder->updateKinematicsForFrame(cpData[icylinder], MBSim::position_cosy);
    fmatvec::Vec3 WrD = cpData[icylinder].getFrameOfReference().getPosition() - WrOP_circle;

    const double &R = circle->getRadius();
    const double &r = cylinder->getRadius();

    fmatvec::Vec3 WbK = circle->getReferenceOrientation().col(2);
    fmatvec::Vec3 WtB = cpData[icylinder].getFrameOfReference().getOrientation().col(1); // normal in first column

    double cos_alpha = WtB.T() * WbK;

    if (nrm2(WrD) > 0) { // conic section theory
      double a = abs(r / cos_alpha);

      fmatvec::Vec3 be1;
      fmatvec::Vec3 be2;
      if (-0.99750 < cos_alpha && cos_alpha < 0.99750) { // until 1 degree tilting
        be2 = crossProduct(WtB, WbK);
        be2 /= nrm2(be2);
        be1 = crossProduct(be2, WbK);
        be1 /= nrm2(be1);
      }
      else {
        if (nrm2(WrD) > 0.01 * a) {
          be2 = WrD / nrm2(WrD);
        }
        else {
          // compute arbitrary normal to cylinder tangent
          be2(0) = WtB(1);
          be2(1) = -WtB(0);
          be2(2) = 0.0;
          be2 /= nrm2(be2);
        }
        be1 = crossProduct(WbK, be2);
        be1 /= nrm2(be1); // b1;
      }

      MBSim::FuncPairEllipseCircle *funcPhi = new MBSim::FuncPairEllipseCircle(R, a, r);

      funcPhi->setDiffVec(WrD);
      funcPhi->setEllipseCOS(be1, be2);

      MBSim::Contact1sSearch searchPhi(funcPhi);
      searchPhi.setSearchAll(true);
      static const int SEC = 16;
      static const double dphi = 2 * M_PI / SEC * 1.02;      // enlargement for better contact convergence
      static const double phiStartSpacing = -2 * M_PI / SEC * 0.02 / 2.;
      searchPhi.setEqualSpacing(SEC, phiStartSpacing, dphi);
      cpData[icylinder].getLagrangeParameterPosition()(1) = searchPhi.slv();
      fmatvec::Vec3 dTilde = funcPhi->computeWrD(cpData[icylinder].getLagrangeParameterPosition()(1));

      delete funcPhi;

      cpData[icircle].getFrameOfReference().getPosition() = WrOP_circle + R * dTilde / nrm2(dTilde);
      cylinder->updateKinematicsForFrame(cpData[icylinder], MBSim::position_cosy);
      fmatvec::Vec3 WrD2 = cpData[icylinder].getFrameOfReference().getPosition() - cpData[icircle].getFrameOfReference().getPosition();

      fmatvec::Vec3 normal = (WrD - WtB.T() * WrD * WtB);

      g(0) = nrm2(WrD2);
      if (nrm2(normal) > 0.01 * a) { // hack of Roland Zander, oh my god; what about the COSY in the non-if-case? TODO
        cpData[icircle].getFrameOfReference().getOrientation().set(0, normal / nrm2(normal));
        cpData[icylinder].getFrameOfReference().getOrientation().set(0, -cpData[icircle].getFrameOfReference().getOrientation().col(0));
        cpData[icylinder].getFrameOfReference().getOrientation().set(2, crossProduct(cpData[icylinder].getFrameOfReference().getOrientation().col(0), cpData[icylinder].getFrameOfReference().getOrientation().col(1)));

        g(0) = cpData[icylinder].getFrameOfReference().getOrientation().col(0).T() * WrD2;
      }
    }
    else { // only dimensioning
      cpData[icircle].getFrameOfReference().setPosition(WrOP_circle);
      cpData[icircle].getFrameOfReference().getOrientation().set(0, -cpData[icylinder].getFrameOfReference().getOrientation().col(0));
      cpData[icircle].getFrameOfReference().getOrientation().set(1, -cpData[icylinder].getFrameOfReference().getOrientation().col(1));
      cpData[icircle].getFrameOfReference().getOrientation().set(2, cpData[icylinder].getFrameOfReference().getOrientation().col(2));

      g(0) = R * abs(cos_alpha) - r;
    }
  }

}

#endif /* _CONTACT_KINEMATICS_CIRCLEHOLLOW_CYLINDERFLEXIBLE_H_ */

