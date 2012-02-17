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

#include "multi_dimensional_fixpoint_solver.h"
#include <iostream>

using namespace fmatvec;
using namespace std;

namespace MBSimNumerics {

  MultiDimensionalFixpointSolver::MultiDimensionalFixpointSolver() :
    criteria(new GlobalShiftCriteriaFunction()), iter(0), itermax(1e3), norms(0), info(1){

  }

  MultiDimensionalFixpointSolver::MultiDimensionalFixpointSolver(Function1<Vec, Vec> *function_) :
      function(function_), criteria(new GlobalShiftCriteriaFunction()), iter(0), itermax(1e3), norms(0), info(1) {
  }

  /*
   * \brief finds a fixpoint starting on the initialGuess values
   * \param intialGuess starting value for the fixpoint iteration
   * \return vector after iteration (solution or currentGuess-value)
   */
  Vec MultiDimensionalFixpointSolver::solve(const Vec & initialGuess) {
    /*Initialise*/
    Vec currentGuess = initialGuess.copy();
    info = 1;
    criteria->setFunction(function);
    criteria->clear();

    for (iter = 0; iter < itermax; iter++) {
      currentGuess = (*function)(currentGuess);

      info = (*criteria)(currentGuess);

      if(info != 1) {
        if(info == -1) { //divergence case //TODO: a more clever structure (with multiple inheritage for shift-functions) might avoid the dynamic casting
          if(dynamic_cast<LocalShiftCriteriaFunction*>(criteria))
            return static_cast<LocalShiftCriteriaFunction*>(criteria)->getLastPoint();
          else if(dynamic_cast<GlobalShiftCriteriaFunction*>(criteria))
            return static_cast<GlobalShiftCriteriaFunction*>(criteria)->getLastPoint();
        }

        return currentGuess;
      }
    }
    return currentGuess;
  }

} /* namespace MBSimNumerics */
