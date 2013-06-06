/* Copyright (C) 2004-2012 MBSim Development Team
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

#ifndef SPHERE_POLYNOMIALFRUSTUM_H_
#define SPHERE_POLYNOMIALFRUSTUM_H_

#include "contact_kinematics.h"

#include <mbsim/contours/polynomial_frustum.h>
#include <mbsim/contours/sphere.h>
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>

namespace MBSim {

  class PolyFurstumSphereContact : public Function1<fmatvec::Vec, fmatvec::Vec> {
    public:
      PolyFurstumSphereContact(const PolynomialFrustum * frustum) :
        frustum(frustum) {
      }

      virtual ~PolyFurstumSphereContact() {
      }

      void setCenter(const fmatvec::Vec3 & sphereCenter_) {
        sphereCenter = sphereCenter_;
      }

      fmatvec::Vec operator()(const fmatvec::Vec &x, const void* = NULL);

    protected:
      /*!
       * \brief center of the sphere
       */
      fmatvec::Vec3 sphereCenter;

      /*!
       * \brief polynomial parameters of the frustum
       */
      const PolynomialFrustum * frustum;
  };

  class PolyFurstumSphereContactJacobian : public NewtonJacobianFunction {
    public:
      PolyFurstumSphereContactJacobian(const PolynomialFrustum * frustum) :
        frustum(frustum) {
      }

      virtual ~PolyFurstumSphereContactJacobian() {
      }

      void setCenter(const fmatvec::Vec3 & sphereCenter_) {
        sphereCenter = sphereCenter_;
      }

      virtual fmatvec::SqrMat operator ()(const fmatvec::Vec & x, const void* = NULL);

    protected:
      /*!
       * \brief center of the sphere
       */
      fmatvec::Vec3 sphereCenter;

      /*!
       * \brief polynomial parameters of the frustum
       */
      const PolynomialFrustum * frustum;
  };

  /*!
   * \brief class for contact kinematics between convex frustum and a sphere
   * \author Kilian Grundl
   * \date  02.05.2013
   */
  class ContactKinematicsSpherePolynomialFrustum : public MBSim::ContactKinematics {
    public:
      ContactKinematicsSpherePolynomialFrustum();
      virtual ~ContactKinematicsSpherePolynomialFrustum();

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec& g, ContourPointData *cpData, int index = 0);
      virtual void updatewb(fmatvec::Vec& wb, const fmatvec::Vec &g, ContourPointData *cpData) {
        throw MBSimError("ERROR (ContactKinematicsAreaPolynomialFrustum::updatewb): not implemented!");
      }

    protected:

      /**
       * \brief contour index of sphere (in cpData)
       */
      int isphere;

      /**
       * \brief contour index of frustum (in cpData)
       */
      int ifrustum;

      /**
       * \brief pointer to the contour class for the sphere
       */
      Sphere *sphere;

      /*!
       * \brief pointer to the contour class for the polynomial frustum
       */
      PolynomialFrustum *frustum;

      /*!
       * \brief Newton solver for nonlinear problem
       */
      MultiDimensionalNewtonMethod newton;

      /*
       * \brief function for contact search
       */
      PolyFurstumSphereContact * func;

      /*!
       * \brief jacobian function for solution
       */
      PolyFurstumSphereContactJacobian* jacobian;

      /*!
       * \brief residual function for newton
       */
      GlobalResidualCriteriaFunction* criteria;

      /*!
       * \brief damping function for newton
       */
      StandardDampingFunction* damping;

      /*!
       * \brief solution vector of nonlinear system
       */
      fmatvec::Vec x;



  };

} /* namespace MBSim */
#endif /* SPHERE_POLYNOMIALFRUSTUM_H_ */
