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

#ifndef NUMERICS_NUMERICSMULTIDIMENSIONALNEWTONMETHOD_H_
#define NUMERICS_NUMERICSMULTIDIMENSIONALNEWTONMETHOD_H_

#include <fmatvec.h>

#include <map>

#include <mbsim/numerics/functions/newton_method_jacobian_functions.h>
#include <mbsim/numerics/functions/criteria_functions.h>
#include <mbsim/numerics/functions/damping_functions.h>

namespace fmatvec {
  bool operator<(const fmatvec::Index & i1, const fmatvec::Index & i2);
}

namespace MBSim {

  /**
   * \brief Newton method for multidimensional root finding
   * \author Kilian Grundl (3.2.2012)
   */
  class MultiDimensionalNewtonMethod {
    public:
      /*!
       * \brief plain constructor
       */
      MultiDimensionalNewtonMethod();

      virtual ~MultiDimensionalNewtonMethod() {
      }

      /* GETTER / SETTER */
      int getNumberOfIterations() const {
        return iter;
      }
      int getNumberOfMaximalIterations() const {
        return itermax;
      }
      int getInfo() const {
        return info;
      }
      void setMaximumNumberOfIterations(int itmax_) {
        itermax = itmax_;
      }
      void setFunction(Function1<fmatvec::Vec, fmatvec::Vec> *function_) {
        function = function_;
      }
      void setJacobianFunction(NewtonJacobianFunction * jacobian_) {
        jacobian = jacobian_;
      }
      void setDampingFunction(DampingFunction * damping_) {
        damping = damping_;
      }
      void setCriteriaFunction(CriteriaFunction * criteria_) {
        criteria = criteria_;
      }
      CriteriaFunction * getCriteriaFunction() {
        return criteria;
      }
      /***************************************************/

      /**
       * \brief solve nonlinear root function
       * \param initialValue initial value
       */
      fmatvec::Vec solve(const fmatvec::Vec & initialValue);

    private:

      /**
       * \brief root function
       */
      Function1<fmatvec::Vec, fmatvec::Vec> *function;

      /**
       * \brief Jacobian matrix
       */
      NewtonJacobianFunction *jacobian;

      /*
       * \brief damping function
       */
      DampingFunction *damping;

      /*
       * \brief criteria function
       *
       * This function defines the criteria when to stop the Newton algorithm
       */
      CriteriaFunction *criteria;

      /**
       * \brief maximum number of iterations, actual number of iterations, maximum number of damping steps, information about success
       */
      int itermax;

      /*
       * \brief number of iterations
       */
      int iter;

      /*
       * \brief information about the result of the method
       */
      int info;

  };
}
#endif /* NUMERICSMULTIDIMENSIONALNEWTONMETHOD_H_ */
