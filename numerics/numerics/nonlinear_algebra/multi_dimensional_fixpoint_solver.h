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

#ifndef NUMERICS_MULTI_DIMENSIONAL_FIXPOINT_SOLVER_H_
#define NUMERICS_MULTI_DIMENSIONAL_FIXPOINT_SOLVER_H_

#include <fmatvec.h>

#include <numerics/functions/function.h>

namespace MBSimNumerics {

  /**
   * \brief Fixpoint-Solver for multi-dimensional fixpoint-finding
   * \author Kilian Grundl
   * \date 2012-02-07 copied from mbsim/utils/nonlinear_algebra
   */
  class MultiDimensionalFixpointSolver {
    public:
      /*!
       * \brief plain constructor
       */
      MultiDimensionalFixpointSolver(){}

      /*
       * \brief constructor
       * \param fct pointer to used fix-point-function
       */
      MultiDimensionalFixpointSolver(Function1<fmatvec::Vec, fmatvec::Vec> *function_);

      virtual ~MultiDimensionalFixpointSolver(){};

      /* GETTER / SETTER */
      /*!
       * \brief returns info of iteration progress
       * info == 0 :  a solution has been found
       * info == -1:  no converge
       * info == 1:   process (seems to) converge but hasn't finished
       */
      void setFunction(Function1<fmatvec::Vec, fmatvec::Vec> *function_) {function = function_;}
      int getInfo() {
        return info;
      }
      std::vector<double> getNorms() {
        return norms;
      }
      double getNumberOfIterations() {
        return iter;
      }
      double getNumberOfMaximalIterations() {
        return itermax;
      }
      void setNumberOfMaximalIterations(int itermax_) {
        itermax = itermax_;
      }
      double getTolerance() {
        return tol;
      }
      void setTolerance(double tol_) {
        tol = tol_;
      }
      /*******************/

      fmatvec::Vec solve(const fmatvec::Vec &initialGuess);

    private:
      /**
       * \brief fix-point function
       */
      Function1<fmatvec::Vec, fmatvec::Vec> *function;

      /**
       * \brief tolerance
       */
      double tol;

      /**
       *  \brief number of iterations
       */
      int iter;

      /**
       * \brief maximal iterations
       */
      double itermax;

      /**
       *  \brief vector of norms
       */
      std::vector<double> norms;

      /**
       *  \brief information variable about success of iteration
       */
      int info;
  };

} /* namespace MBSimNumerics */
#endif /* MULTI_DIMENSIONAL_FIXPOINT_SOLVER_H_ */
