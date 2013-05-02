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
#include <mbsim/contours/area.h>
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>

namespace MBSim {

  class edgePolyFrustum : public Function1<fmatvec::Vec, fmatvec::Vec> {
    public:
      /*!
       * \brief constructor
       */
      edgePolyFrustum(const PolynomialFrustum * frustum);

      /*!
       * \brief standard destructor
       */
      virtual ~edgePolyFrustum();

      /*!
       * \brief set the one corner point A and the direction of the edge with (A + dir) is the other corner point
       */
      void setAdir(const fmatvec::Vec3 & A, const fmatvec::Vec3 & dir);

      fmatvec::Vec operator()(const fmatvec::Vec &x, const void* = NULL);

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
      virtual ~edgePolyFrustumCriteria(){};

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
      virtual int operator ()(const fmatvec::Vec & x, const void * = NULL);
      virtual bool isBetter(const fmatvec::Vec & x);
      virtual void clear(){criteriaResults.clear();}
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
      double frustumHeight;

      /**
       * \brief saves the results of the criteria
       */
      std::vector<double> criteriaResults;

      /*!
       * \brief xPosition of starting Point
       */
      double ax;

      /*!
       * \brief xdirection
       */
      double dx;
  };

  /*!
   * \brief function that is zero for a height-coordinate of the polynomial frustum on which the normal on that point points towards the given outer point
   */
  class projectAlongNormal : public Function1<fmatvec::Vec, fmatvec::Vec> {
    public:
      projectAlongNormal(PolynomialFrustum * frustum);

      virtual ~projectAlongNormal();

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
  class projectAlongNormalJacobian : public NewtonJacobianFunction {
    public:
      projectAlongNormalJacobian(PolynomialFrustum * frustum);

      virtual ~projectAlongNormalJacobian();

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
   * \brief class for contact kinematics between convex frustum and an area
   * \author Kilian Grundl, Tingting Sun
   * \date  09.10.2012
   */

  class ContactKinematicsAreaPolynomialFrustum : public MBSim::ContactKinematics {
    public:
      ContactKinematicsAreaPolynomialFrustum();
      virtual ~ContactKinematicsAreaPolynomialFrustum();

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec& g, ContourPointData *cpData, int index);
      virtual void updatewb(fmatvec::Vec& wb, const fmatvec::Vec &g, ContourPointData *cpData) {
        throw MBSimError("ERROR (ContactKinematicsAreaPolynomialFrustum::updatewb): not implemented!");
      }

    protected:
      /*!
       * \brief set the values for the contact kinematics for the frustum due to the given x and phi
       */
      void setFrustumOrienationKinematics(const double & x, const double & phi, ContourPointData * cpData);

      /***************************************************/
      /*!
       * \brief check if there is a contact point within the area between the frustum and the area
       * \return true (if) or false (if there is no contact within the area)
       *
       * If there is contact the position and the cpData information is setted right away
       */
      bool cpLocationInArea(fmatvec::Vec & g, ContourPointData * cpData);

      /*!
       * \brief checks if there is a contact point at one of the corner points
       * \return true (if) or false (if there is no contact at one of the corner points)
       */
      bool cornerContact(fmatvec::Vec & g, ContourPointData * cpData);

      /*!
       * \brief checks if there is a contact on one edge of the area
       * \return true (if) or false (if there is no contact at one of the corner points)
       *
       * \todo: unefficient and only finding (one) intersection point --> There should always be two intersection points and then using the middle or something
       */
      bool edgeContact(fmatvec::Vec & g, ContourPointData * cpData);

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
       * \param n normal vector of the area
       */
      int checkPossibleContactPoint(const double & x, const fmatvec::Vec3 & n);

      /*!
       * \brief compute distance between given point and area
       * \param point Given point in world coordinates
       */
      double distance2Area(const fmatvec::Vec3 & point);

      /**
       * \brief contour index of area (in cpData)
       */
      int iarea;

      /**
       * \brief contour index of frustum (in cpData)
       */
      int ifrustum;

      /**
       * \brief pointer to the contour class for the area
       */
      Area *area;

      /*!
       * \brief pointer to the contour class for the polynomial frustum
       */
      PolynomialFrustum *frustum;

      /*!
       * \brief save last value to use it again as starting value for equation 1
       */
      double x1;

      /*!
       * \brief save last value to use it again as starting value for equation 2
       */
      double x2;

      /*!
       * \brief array of the four corner points of the area in the frame of the frustum
       */
      fmatvec::Vec3 cornerPoints[4];

      /*!
       * \brief function for intersection point
       */
      projectAlongNormal * funcProjectAlongNormal;

      /*!
       * \brief newton method for solving the edge contact
       */
      MultiDimensionalNewtonMethod newtonProjectAlongNormal;

      /*!
       * \brief Jacobian for newton method
       */
      projectAlongNormalJacobian * jacobianProjectAlongNormal;

      /*!
       * \brief criteria for newton method
       */
      GlobalResidualCriteriaFunction criteriaProjectAlongNormal;

      /*!
       * \brief damping function for newton method
       */
      StandardDampingFunction dampingProjectAlongNormal;

      /*!
       * \brief function for intersection point
       */
      edgePolyFrustum * funcEdge;

      /*!
       * \brief newton method for solving the edge contact
       */
      MultiDimensionalNewtonMethod newtonEdge;

      /*!
       * \brief Jacobian for newton method
       */
      NumericalNewtonJacobianFunction jacobianEdge;

      /*!
       * \brief criteria for newton method
       */
      edgePolyFrustumCriteria criteriaEdge;

      /*!
       * \brief damping function for newton method
       */
      StandardDampingFunction dampingEdge;

      /*!
       * \brief index of last edge contact
       */
      int ilast;

      /*!
       * \brief last position of edge contact
       */
      fmatvec::Vec xi;

  };

} /* namespace MBSim */
#endif /* AREA_POLYNOMIALFRUSTUM_H_ */
