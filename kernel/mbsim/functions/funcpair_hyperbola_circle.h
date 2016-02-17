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

#ifndef _FUNCPAIR_HYPERBOLA_CIRCLE_H_
#define _FUNCPAIR_HYPERBOLA_CIRCLE_H_

#include <mbsim/functions/funcpair_conesection_circle.h>

namespace MBSim {

  /*!
   * \brief root function for planar pairing Hyperbola and Circle
   * \author Bastian Esefeld
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairHyperbolaCircle : public FuncPairConeSectionCircle {
    public:
      /*!
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \default conesection in circle
       */
      FuncPairHyperbolaCircle(double R_, double a_, double b_) :
          FuncPairConeSectionCircle(R_, a_, b_) {
      }

      /*!
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \param conesection in circle
       */
      FuncPairHyperbolaCircle(double R_, double a_, double b_, bool hy_IN_ci_) :
          FuncPairConeSectionCircle(R_, a_, b_, hy_IN_ci_) {
      }

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &phi);
      fmatvec::Vec3 getWrD(const double &phi);
      /*************************************************/
  };

  inline double FuncPairHyperbolaCircle::operator()(const double &phi) {
    return -2 * b * (b2(0) * d(0) + b2(1) * d(1) + b2(2) * d(2)) * cosh(phi) - 2 * a * (b1(0) * d(0) + b1(1) * d(1) + b1(2) * d(2)) * sinh(phi) - ((a * a) + (b * b)) * sinh(2 * phi);
  }
  inline fmatvec::Vec3 FuncPairHyperbolaCircle::getWrD(const double &phi) {
    return d + b1 * a * cosh(phi) + b2 * b * sinh(phi);
  }

}

#endif
