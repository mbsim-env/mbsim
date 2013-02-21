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
      edgePolyFrustum(const PolynomialFrustum * frustum);

      virtual ~edgePolyFrustum();

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
      void setFrustumOrienationKinematics(const double & x, const double & phi, fmatvec::Vec & g, ContourPointData * cpData);

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
       * \brief given two vectors x and v, this function is used to generate the rotation matrix which rotating the current reference frame to a new one
       *        the new frame keeps x as an axis and uses the cross product of x and v as new z axis
       * \return orientation matrix A so that for a point P, P_newframe=A*(P_currentframe-vec(from orgin of current to new frame))
       *
       * \todo: activate function to use in case that the contact point is on the edge
       */
//      fmatvec::SqrMat3 RotatM_2vec(fmatvec::Vec3 X, fmatvec::Vec3 V);
      /*!
       * \brief this function is used to find the smallest distance between a point and the surface of a frustum
       *        To solve this, we transform the 3D problem to a 2D problem on a section which includes two lines:
       *        one goes through the origin of the frustum (center point of bottom or top circle) and the given point;
       *        the other is the x-axis
       * the coordinates used here is under      which??  frame
       *
       * * \todo: activate function to use in case that the contact point is on the edge
       */
      //     CP_item_frustum CP_toP_onfrustum3D(fmatvec::Vec3 P);
      /*!
       * \brief this function is used to search the smallest distance between the surface of a frustum and a linesegment with end points P1,P2
       *        We will use numerical scheme, first select several points on the line segment and calculate the distances, then around the point with the
       *        smallest distance, discretize the neighboring part with much denser points and then calculate the distance again.
       */

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
      edgePolyFrustum * funcAB;

      /*!
       * \brief newton method for solving the edge contact
       */
      MultiDimensionalNewtonMethod newtonedge;

      /*!
       * \brief Jacobian for newton method
       */
      NumericalNewtonJacobianFunction jacobian;

      /*!
       * \brief criteria for newton method
       */
      edgePolyFrustumCriteria criteria;

      /*!
       * \brief damping function for newton method
       */
      StandardDampingFunction damping;

      /*!
       * \brief index of last edge contact
       */
      int ilast;

      /*!
       * \brief last position of edge contact
       */
      fmatvec::Vec xi;

  };
/*!
 * \brief this class stores information about contact points both on the surface of frustum and the other item as well as the smallest distance
 */
//  class CP_item_frustum {
//    private:
//      fmatvec::Vec3 P_fru;
//      fmatvec::Vec3 P_other;
//      double dis;
//    public:
//      CP_item_frustum(const fmatvec::Vec3 _P_fru, const fmatvec::Vec3 _P_other, double _dis) {
//        P_fru = _P_fru;
//        P_other = _P_other;
//        dis = _dis;
//      }
//      ;
//      ~CP_item_frustum();
//  };
} /* namespace MBSim */
#endif /* AREA_POLYNOMIALFRUSTUM_H_ */
