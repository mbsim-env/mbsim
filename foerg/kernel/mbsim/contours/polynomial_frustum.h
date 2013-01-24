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

#ifndef POLYNOMIAL_FRUSTUM_H_
#define POLYNOMIAL_FRUSTUM_H_

#include <mbsim/contour.h>

#include <mbsim/utils/colors.h>

#include <fmatvec.h>

using namespace fmatvec;

namespace MBSim {


  /*!
   * \brief Frustum contour with a polynomial radius over height course
   * \author Kilian Grundl
   * \date  09.10.2012
   */

  class PolynomialFrustum: public MBSim::RigidContour {
    public:
      /*!
       * \brief Constructor
       * \param name Name of the contour
       */
      PolynomialFrustum(const std::string & name);

      /*!
       * \brief Destructor
       */
      virtual ~PolynomialFrustum();

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "PolynomialFrustum"; }
      virtual void init(InitStage stage);
      /***************************************************/

      /*!
       * \brief Set polynomial parameters like a+bx+cx^2+dx^3+...
       * \param parameters vector holding the parameters starting with a
       */
      void setPolynomialParameters(const std::vector<double> & parameters_);

      /*!
       * \brief set hieght of frustum
       * \param height height of the frustum
       */
      void setHeight(const double & height_);
      double getHeight(){
        return height;
      }

      /*!
       * \brief get the radius of the circumsphere
       */
      double getRadiSphere();

#ifdef HAVE_OPENMBVCPPINTERFACE
      /*!
       * \brief enable visualisation
       * \param enable           enable or disable openmbv
       * \param polynomialPoints how fine should the grid be in polynomial direction
       * \param circulaPoints    how fine should the grid be in circular direction
       */
      void enableOpenMBV(bool enable = true, int polynomialPoints = 0, int circularPoints = 25);

      /*!
       * \brief set color of body
       */
      void setColor(const RGBColor & color);

      /*!
       * \brief set transparency of body
       */
      void setTransparency(const double & transparency);
#endif

    protected:
      /*!
       * \brief vector holding our parameters of the polynom describing the frustum (radius over height)
       */
      std::vector<double> parameters;

      /*!
       * \brief height of the frustum
       */
      double height;

      /*!
       * \brief get value at position of 0 derivative
       */
      double getValue(const double & x);

      /*!
       * \brief get value at position of first derivative
       */
      double getValueD1(const double & x);

      /*!
       * \brief get value at position of second derivative
       */
      double getValueD2(const double & x);

      /*!
       * \brief get inner peak value of the polynomial
       */
      double getXPolyMax();


      /*!
       * \brief get coefficient vector of the polynomial
       */
      std::vector<double> getPolynomialParameters(){return parameters;};
#ifdef HAVE_OPENMBVCPPINTERFACE
      /*!
       * \brief color values for the iv-body
       */
      RGBColor color;

      /*!
       * \brief transparency value for the body
       */
      double transparency;

      /*!
       * \brief grid points in polynomial direction
       */
      int polynomialPoints;

      /*!
       * \brief grid points in polynomial direction
       */
      int circularPoints;

      /*!
       * \brief create inventor file for visualisation
       */
      void createInventorFile();

#endif
  };

}

#endif /* POLYNOMIAL_FRUSTUM_H_ */
