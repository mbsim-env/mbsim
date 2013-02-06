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

#include <mbsim/numerics/functions/criteria_functions.h>

namespace MBSim {

  /**
   * \brief Fixpoint-Solver for multi-dimensional fixpoint-finding
   * \author Kilian Grundl
   * \date 2012-02-07 copied from mbsim/utils/nonlinear_algebra
   */
  template <class VecType, class AT>
  class MultiDimensionalFixpointSolver {
    public:
      /*!
       * \brief plain constructor
       */
      MultiDimensionalFixpointSolver();

      /*
       * \brief constructor
       * \param fct pointer to used fix-point-function
       */
      MultiDimensionalFixpointSolver(Function1<VecType, VecType> *function_);

      virtual ~MultiDimensionalFixpointSolver() {
      }
      ;

      /* GETTER / SETTER */
      /*!
       * \brief returns info of iteration progress
       * info == 0 :  a solution has been found
       * info == -1:  no converge
       * info == 1:   process (seems to) converge but hasn't finished
       */
      void setFunction(Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > *function_) {
        this->function = function_;
      }
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
      void setCriteriaFunction(CriteriaFunction<VecType, AT> * criteria_) {
        this->criteria = criteria_;
      }
      /*******************/

      fmatvec::Vector<VecType, AT> solve(const fmatvec::Vector<VecType, AT> &initialGuess);

    private:
      /**
       * \brief fixpoint function
       */
      Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > *function;

      /*
       * \brief criteria function
       *
       * This function defines the criteria when to stop the fixpoint iteration
       */
      CriteriaFunction<VecType, AT> *criteria;

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

  template <class VecType, class AT>
  MultiDimensionalFixpointSolver<VecType, AT>::MultiDimensionalFixpointSolver() :
      function(0), criteria(0), iter(0), itermax(1e3), norms(0), info(1) {

  }

  template <class VecType, class AT>
  MultiDimensionalFixpointSolver<VecType, AT>::MultiDimensionalFixpointSolver(Function1<VecType, VecType> *function_) :
      function(function_), criteria(0), iter(0), itermax(1e3), norms(0), info(1) {
  }

  /*
   * \brief finds a fixpoint starting on the initialGuess values
   * \param intialGuess starting value for the fixpoint iteration
   * \return vector after iteration (solution or currentGuess-value)
   */
  template <class VecType, class AT>
  fmatvec::Vector<VecType, AT> MultiDimensionalFixpointSolver<VecType, AT>::solve(const fmatvec::Vector<VecType, AT> & initialGuess) {
    /*Initialise*/
    fmatvec::Vector<VecType, AT> currentGuess = initialGuess;
    info = 1;
    criteria->setFunction(function);
    criteria->clear();

    for (iter = 0; iter < itermax; iter++) {
      currentGuess = (*function)(currentGuess);

      info = (*criteria)(currentGuess);

      if (info != 1) {
        if (info == -1) { //divergence case //TODO: a more clever structure (with multiple inheritage for shift-functions) might avoid the dynamic casting
          if (dynamic_cast<LocalShiftCriteriaFunction<VecType, AT>*>(criteria))
            return static_cast<LocalShiftCriteriaFunction<VecType, AT>*>(criteria)->getLastPoint();
          else if (dynamic_cast<GlobalShiftCriteriaFunction<VecType, AT>*>(criteria))
            return static_cast<GlobalShiftCriteriaFunction<VecType, AT>*>(criteria)->getLastPoint();
        }

        return currentGuess;
      }
    }
    return currentGuess;
  }

} /* namespace MBSim */
#endif /* MULTI_DIMENSIONAL_FIXPOINT_SOLVER_H_ */
