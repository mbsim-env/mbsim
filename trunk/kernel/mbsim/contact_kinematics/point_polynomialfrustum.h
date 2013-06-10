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

#ifndef POINT_POLYNOMIALFRUSTUM_H_
#define POINT_POLYNOMIALFRUSTUM_H_

#include "contact_kinematics.h"

#include <mbsim/contours/polynomial_frustum.h>
#include <mbsim/contours/point.h>
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>

namespace MBSim {

  /*!
   * \brief function that is zero for a height-coordinate of the polynomial frustum on which the normal on that point points towards the given outer point
   */
  class projectPointAlongNormal : public Function1<fmatvec::Vec, fmatvec::Vec> {
    public:
      projectPointAlongNormal(PolynomialFrustum * frustum);

      virtual ~projectPointAlongNormal();

      void setUpSystemParamters(const fmatvec::Vec3 & referencePoint, const double & phi);

      fmatvec::Vec operator()(const fmatvec::Vec &x, const void* = NULL);

    protected:
      /*!
       * \brief constant pointer to the frustum
       */
      PolynomialFrustum * frustum;

      /*!
       * \brief point that should be projected
       */
      fmatvec::Vec3 referencePoint;

      /*!
       * \brief azimuathal position where it all happens
       */
      double phi;
  };

  /*!
   * \brief the Jacobian function for the projectAlongNormal Function
   */
  class projectPointAlongNormalJacobian : public NewtonJacobianFunction {
    public:
      projectPointAlongNormalJacobian(PolynomialFrustum * frustum);

      virtual ~projectPointAlongNormalJacobian();

      void setUpSystemParamters(const fmatvec::Vec3 & referencePoint, const double & phi);

      fmatvec::SqrMat operator()(const fmatvec::Vec &x, const void* = NULL);

    protected:
      /*!
       * \brief constant pointer to the frustum
       */
      PolynomialFrustum * frustum;

      /*!
       * \brief point that should be projected
       */
      fmatvec::Vec3 referencePoint;

      /*!
       * \brief azimuathal position where it all happens
       */
      double phi;
  };


  /*!
   * \brief class for contact kinematics between convex frustum and an rectangle
   * \author Kilian Grundl, Tingting Sun
   * \date  06.06.2013
   */

  class ContactKinematicsPointPolynomialFrustum : public MBSim::ContactKinematics {
    public:
      ContactKinematicsPointPolynomialFrustum();
      virtual ~ContactKinematicsPointPolynomialFrustum();

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec& g, ContourPointData *cpData, int index = 0);
      virtual void updatewb(fmatvec::Vec& wb, const fmatvec::Vec &g, ContourPointData *cpData) {
        throw MBSimError("ERROR (ContactKinematicsRectanglePolynomialFrustum::updatewb): not implemented!");
      }

    protected:
      /*!
       * \brief set the values for the contact kinematics for the frustum due to the given x and phi
       */
      void setFrustumOrienationKinematics(const double & x, const double & phi, ContourPointData * cpData);

      /**
       * \brief contour index of rectangle (in cpData)
       */
      int ipoint;

      /**
       * \brief contour index of frustum (in cpData)
       */
      int ifrustum;

      /**
       * \brief pointer to the contour class for the rectangle
       */
      Rectangle *point;

      /*!
       * \brief pointer to the contour class for the polynomial frustum
       */
      PolynomialFrustum *frustum;

      /*!
       * \brief function for intersection point
       */
      projectPointAlongNormal * funcProjectAlongNormal;

      /*!
       * \brief newton method for solving the edge contact
       */
      MultiDimensionalNewtonMethod newtonProjectAlongNormal;

      /*!
       * \brief Jacobian for newton method
       */
      projectPointAlongNormalJacobian * jacobianProjectAlongNormal;

      /*!
       * \brief criteria for newton method
       */
      GlobalResidualCriteriaFunction criteriaProjectAlongNormal;

      /*!
       * \brief damping function for newton method
       */
      StandardDampingFunction dampingProjectAlongNormal;

  };

} /* namespace MBSim */
#endif /* POINT_POLYNOMIALFRUSTUM_H_ */
