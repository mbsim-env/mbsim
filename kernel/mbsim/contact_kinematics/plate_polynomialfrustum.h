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

#ifndef AREA_POLYNOMIALFRUSTUM_H_
#define AREA_POLYNOMIALFRUSTUM_H_

#include "contact_kinematics.h"

#include <mbsim/contours/polynomial_frustum.h>
#include <mbsim/contours/plate.h>
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>
#include <mbsim/contact_kinematics/point_polynomialfrustum.h>

namespace MBSim {

  /*!
   * \brief Function describing the scalar product between normal of frustum point and difference between frustum point and point on line defined by one parameter t
   */
  class edgePolyFrustum : public Function<fmatvec::Vec(fmatvec::Vec)> {
    public:
      /*!
       * \brief constructor
       */
      edgePolyFrustum(const PolynomialFrustum * frustum_);

      /*!
       * \brief standard destructor
       */
      ~edgePolyFrustum() override;

      /*!
       * \brief set the one corner point A and the direction of the edge with (A + dir) is the other corner point
       */
      void setAdir(const fmatvec::Vec3 & A_, const fmatvec::Vec3 & dir_);

      fmatvec::Vec operator()(const fmatvec::Vec &xin) override;

    protected:
      /*!
       * \brief polynomials of frustum contour
       */
      const PolynomialFrustum * frustum;

      /*!
       * \brief position of starting point on the line
       */
      fmatvec::Vec A;

      /*!
       * \brief direction of edge
       */
      fmatvec::Vec dir;
  };

  class edgePolyFrustumCriteria : public CriteriaFunction {
    public:
      /**
       * \brief Constructor
       */
      edgePolyFrustumCriteria(const double & tolerance_ = 1e-10);

      /**
       * \brief Destructor
       */
      ~edgePolyFrustumCriteria() override= default;;

      /*!
       * \brief set frustum height
       */
      void setFrustumHeight(const double & frustumHeight_) {
        frustumHeight = frustumHeight_;
      }

      /*!
       * \brief set starting x coordinate
       */
      void setStartingXCoordinate(const double & ax_) {
        ax = ax_;
      }

      /*!
       * \brief set starting x coordinate
       */
      void setdirectionXCoordinate(const double & dx_) {
        dx = dx_;
      }

      /* INHERITED INTERFACE */
      int operator ()(const fmatvec::Vec &xin) override;
      bool isBetter(const fmatvec::Vec &x, const fmatvec::Vec & fVal = fmatvec::Vec(0,fmatvec::NONINIT)) override;
      void clear() override{criteriaResults.clear();}
      /*END - INHERITED INTERFACE*/

      const std::vector<double> & getResults() {
        return criteriaResults;
      }

    protected:
      /*!
       * \brief checks if current point fullfills the boundary conditions
       */
      bool inBounds(const double & t);
      /**
       * \brief tolerance value for the criteria results
       */
      double tolerance;

      /*!
       * \brief height of frustum
       */
      double frustumHeight{-1};

      /**
       * \brief saves the results of the criteria
       */
      std::vector<double> criteriaResults;

      /*!
       * \brief xPosition of starting Point
       */
      double ax{0.};

      /*!
       * \brief xdirection
       */
      double dx{0.};
  };

  /*!
   * \brief class for contact kinematics between convex frustum and an plate
   * \author Kilian Grundl, Tingting Sun
   * \date  09.10.2012
   */

  class ContactKinematicsPlatePolynomialFrustum : public MBSim::ContactKinematics {
    public:
      ContactKinematicsPlatePolynomialFrustum();
      ~ContactKinematicsPlatePolynomialFrustum() override;

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<Contour*> &contour) override;
      void updateg(SingleContact &contact, int i=0) override;

      void setGridSizeY(int gridSizeY_);
      void setGridSizeZ(int gridSizeZ_);

    protected:
      /*!
       * \brief set the values for the contact kinematics for the frustum due to the given x and phi
       */
      void setFrustumOrienationKinematics(const double & x, const double & phi, SingleContact &contact);

      /***************************************************/
      /*!
       * \brief check if there is a contact point within the plate between the frustum and the plate
       * \return true (if) or false (if there is no contact within the plate)
       *
       * If there is contact the position and the cpData information is setted right away
       */
      bool cpLocationInPlate(SingleContact &contact);

      /*!
       * \brief if the unique contact point cannot be found a grid is walked through and a weighted sum results in the contact point
       * \return true (if) or false (if there is no contact at one of the corner points)
       */
      bool gridContact(SingleContact &contact);

      /*!
       * \brief checks if there is a contact point at one of the corner points
       * \return true (if) or false (if there is no contact at one of the corner points)
       */
      bool cornerContact(SingleContact &contact);

      /*!
       * \brief checks if there is a contact on one edge of the plate
       * \return true (if) or false (if there is no contact at one of the corner points)
       *
       * \todo: unefficient and only finding (one) intersection point --> There should always be two intersection points and then using the middle or something
       */
      bool edgeContact(SingleContact &contact);

      /*!
       * \brief computes the point on the contour of the frustum due to the height-coordinate x and the normal in world coordinates
       * \param x height coordinate
       * \param n normal of the frustum
       * \return contour point in frustum coordinates of the contour point on the frustum
       */
      fmatvec::Vec3 computeContourPointFrustum(const double & x, const fmatvec::Vec3 & n);

      /*!
       * \brief computes the point on the contour of the frustum due to the height-coordinate x and the normal in world coordinates
       * \param x height coordinate
       * \param n normal of the frustum
       * \return contour point in world coordinates of the contour point on the frustum
       */
      fmatvec::Vec3 computeContourPoint(const double & x, const fmatvec::Vec3 & n);

      /*!
       * \brief check if frustum point identified by coordinate x is possible contact point
       * \param x height-coordinate of possible contact point
       * \param n normal vector of the plate
       */
      int checkPossibleContactPoint(const double & x, const fmatvec::Vec3 & n);

      /*!
       * \brief compute distance between given point and plate
       * \param point Given point in world coordinates
       */
      double distance2Plate(const fmatvec::Vec3 & point);

      /*!
       * \brief updates the grid for the discrete contact point approximation
       */
      void updateGrid();

      /**
       * \brief contour index of plate (in cpData)
       */
      int iplate{-1};

      /**
       * \brief contour index of frustum (in cpData)
       */
      int ifrustum{-1};

      /**
       * \brief pointer to the contour class for the plate
       */
      Plate *plate{nullptr};

      /*!
       * \brief pointer to the contour class for the polynomial frustum
       */
      PolynomialFrustum *frustum{nullptr};

      /*!
       * \brief sign of height-direction
       */
      double signh{1.};

      /*!
       * \brief grid size in y-direction for the search with grid points
       */
      int gridSizeY{5};

      /*!
       * \brief grid size in z-direction for the search with grid points
       */
      int gridSizeZ{5};

      /*!
       * \brief save last value to use it again as starting value for equation 1
       */
      double x1{-1};

      /*!
       * \brief save last value to use it again as starting value for equation 2
       */
      double x2{-1};

      /*!
       * \brief array of the four corner points of the plate in the frame of the frustum
       */
      fmatvec::Vec3 cornerPoints[4];

      /*!
       * \brief saves the points for the contact point computation
       */
      std::vector<std::vector<fmatvec::Vec3> > gridPoints;

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

      /*!
       * \brief function for intersection point
       */
      edgePolyFrustum * funcEdge{nullptr};

      /*!
       * \brief newton method for solving the edge contact
       */
      MBSim::MultiDimensionalNewtonMethod newtonEdge;

      /*!
       * \brief Jacobian for newton method
       */
      MBSim::NumericalNewtonJacobianFunction jacobianEdge;

      /*!
       * \brief criteria for newton method
       */
      edgePolyFrustumCriteria criteriaEdge;

      /*!
       * \brief damping function for newton method
       */
      MBSim::StandardDampingFunction dampingEdge;

      /*!
       * \brief index of last edge contact
       */
      int ilast{-1};

      /*!
       * \brief last position of edge contact
       */
      fmatvec::Vec xi;

      fmatvec::Vec2 zeta;
  };

} /* namespace MBSim */
#endif /* AREA_POLYNOMIALFRUSTUM_H_ */
