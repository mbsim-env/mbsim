/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef _FUNCPAIR_ELLIPSE_CIRCLE_H_
#define _FUNCPAIR_ELLIPSE_CIRCLE_H_

#include <mbsim/functions/contact/funcpair_conesection_circle.h>

namespace MBSim {

  /*!
   * \brief root function for planar pairing Ellipse and Circle
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairEllipseCircle : public FuncPairConeSectionCircle {
    public:
      /*!
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \default conesection in circle
       */
      FuncPairEllipseCircle(double R_, double a_, double b_) :
          FuncPairConeSectionCircle(R_, a_, b_) {
      }

      /*!
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \param conesection in circle
       */
      FuncPairEllipseCircle(double R_, double a_, double b_, bool el_IN_ci_) :
          FuncPairConeSectionCircle(R_, a_, b_, el_IN_ci_) {
      }

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &phi) override;
      fmatvec::Vec3 evalWrD(const double &phi) override;
      /*************************************************/

      /* GETTER / SETTER */
      void setEllipseCOS(fmatvec::Vec3 b1e_, fmatvec::Vec3 b2e_);
      /*************************************************/
  };

  inline void FuncPairEllipseCircle::setEllipseCOS(fmatvec::Vec3 b1e_, fmatvec::Vec3 b2e_) {
    setSectionCOS(b1e_, b2e_);
  }
  inline double FuncPairEllipseCircle::operator()(const double &phi) {
    return -2 * b * (b2(0) * d(0) + b2(1) * d(1) + b2(2) * d(2)) * cos(phi) + 2 * a * (b1(0) * d(0) + b1(1) * d(1) + b1(2) * d(2)) * sin(phi) + ((a * a) - (b * b)) * sin(2 * phi);
  }
  inline fmatvec::Vec3 FuncPairEllipseCircle::evalWrD(const double &phi) {
    return d + b1 * a * cos(phi) + b2 * b * sin(phi);
  }

}

#endif
