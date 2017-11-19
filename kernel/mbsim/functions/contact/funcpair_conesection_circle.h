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

#ifndef _FUNCPAIR_CONESECTION_CIRCLE_H_
#define _FUNCPAIR_CONESECTION_CIRCLE_H_

#include <mbsim/functions/contact/distance_function.h>

namespace MBSim {

  /*!
   * \brief base root function for planar pairing ConeSection and Circle
   * \author Thorsten Schindler
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class FuncPairConeSectionCircle : public DistanceFunction<double(double)> {
    public:
      /*!
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \default conesection in circle
       */
      FuncPairConeSectionCircle(double R_,double a_,double b_) : R(R_), a(a_), b(b_), sec_IN_ci(true) {}

      /*!
       * \brief constructor
       * \param radius of circle
       * \param length of first semi-axis
       * \param length of second semi-axis
       * \param conesection in circle
       */
      FuncPairConeSectionCircle(double R_, double a_, double b_, bool sec_IN_ci_) :
          R(R_), a(a_), b(b_), sec_IN_ci(sec_IN_ci_) {
      }

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &phi) override = 0;
      double operator[](const double &phi) override;
      fmatvec::Vec3 evalWrD(const double &phi) override = 0;
      /*************************************************/

      /* GETTER / SETTER */
      void setDiffVec(fmatvec::Vec3 d_);

      void setSectionCOS(fmatvec::Vec3 b1_, fmatvec::Vec3 b2_);
      /*************************************************/

    protected:
      /**
       * \brief radius of circle as well as length in b1- and b2-direction
       */
      double R, a, b;

      /**
       * \brief cone-section in circle
       */
      bool sec_IN_ci;

      /**
       * \brief normed base-vectors of cone-section
       */
      fmatvec::Vec3 b1, b2;

      /**
       * \brief distance-vector of cone-section- and circle-midpoint
       */
      fmatvec::Vec3 d;
  };

  inline void FuncPairConeSectionCircle::setDiffVec(fmatvec::Vec3 d_) {
    d = d_;
  }
  inline void FuncPairConeSectionCircle::setSectionCOS(fmatvec::Vec3 b1_, fmatvec::Vec3 b2_) {
    b1 = b1_;
    b2 = b2_;
  }
  inline double FuncPairConeSectionCircle::operator[](const double &phi) {
    if (sec_IN_ci)
      return R - nrm2(evalWrD(phi));
    else
      return nrm2(evalWrD(phi)) - R;
  }

}

#endif
