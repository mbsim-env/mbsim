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

namespace MBSim {

  /*!
   * \brief class for contact kinematics between convex frustum and an area
   * \author Kilian Grundl
   * \date  09.10.2012
   */

  class ContactKinematicsAreaPolynomialFrustum: public MBSim::ContactKinematics {
    public:
      ContactKinematicsAreaPolynomialFrustum();
      virtual ~ContactKinematicsAreaPolynomialFrustum();

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      void assignContours(const std::vector<Contour*> &contour);
      virtual void updateg(fmatvec::Vec& g, ContourPointData *cpData, int index);
      virtual void updatewb(fmatvec::Vec& wb, const fmatvec::Vec &g,ContourPointData *cpData) { throw MBSimError("ERROR (ContactKinematicsAreaPolynomialFrustum::updatewb): not implemented!"); }
      /***************************************************/

    protected:
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

  };

} /* namespace MBSim */
#endif /* AREA_POLYNOMIALFRUSTUM_H_ */
