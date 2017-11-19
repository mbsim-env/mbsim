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
  class projectPointAlongNormal : public Function<fmatvec::Vec(fmatvec::Vec)> {
    public:
      projectPointAlongNormal(PolynomialFrustum * frustum);

      ~projectPointAlongNormal() override;

      void setUpSystemParamters(const fmatvec::Vec3 & referencePoint_, const double & phi_);

      fmatvec::Vec operator()(const fmatvec::Vec &xin) override;

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

      ~projectPointAlongNormalJacobian() override;

      void setUpSystemParamters(const fmatvec::Vec3 & referencePoint_, const double & phi_);

      fmatvec::SqrMat operator()(const fmatvec::Vec &xin) override;

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
   * \brief class for contact kinematics between convex frustum and an plate
   * \author Kilian Grundl, Tingting Sun
   * \date  06.06.2013
   */

  class ContactKinematicsPointPolynomialFrustum : public MBSim::ContactKinematics {
    public:
      ContactKinematicsPointPolynomialFrustum();
      ~ContactKinematicsPointPolynomialFrustum() override;

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<Contour*> &contour) override;
      void updateg(double& g, std::vector<ContourFrame*> &cFrame, int index = 0) override;
      void updatewb(fmatvec::Vec& wb, double g, std::vector<ContourFrame*> &cFrame) override {
        throw MBSimError("(ContactKinematicsPointPolynomialFrustum::updatewb): not implemented!");
      }

    protected:
      /**
       * \brief contour index of plate (in cpData)
       */
      int ipoint{-1};

      /**
       * \brief contour index of frustum (in cpData)
       */
      int ifrustum{-1};

      /**
       * \brief pointer to the contour class for the plate
       */
      Point *point{nullptr};

      /*!
       * \brief pointer to the contour class for the polynomial frustum
       */
      PolynomialFrustum *frustum{nullptr};

      /*!
       * \brief sign of h-direction of frustum
       */
      double signh{1};

      /*!
       * \brief function for intersection point
       */
      MBSim::projectPointAlongNormal * funcProjectAlongNormal{nullptr};

      /*!
       * \brief newton method for solving the edge contact
       */
      MBSim::MultiDimensionalNewtonMethod newtonProjectAlongNormal;

      /*!
       * \brief Jacobian for newton method
       */
      MBSim::projectPointAlongNormalJacobian * jacobianProjectAlongNormal{nullptr};

      /*!
       * \brief criteria for newton method
       */
      GlobalResidualCriteriaFunction criteriaProjectAlongNormal;

      /*!
       * \brief damping function for newton method
       */
      MBSim::StandardDampingFunction dampingProjectAlongNormal;

  };

} /* namespace MBSim */
#endif /* POINT_POLYNOMIALFRUSTUM_H_ */
