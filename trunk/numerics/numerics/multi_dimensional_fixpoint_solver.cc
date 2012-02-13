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

using namespace fmatvec;

namespace MBSimNumerics {

  MultiDimensionalFixpointSolver::MultiDimensionalFixpointSolver(Function1<Vec, Vec> *function_) :
      function(function_), tol(1e-10), iter(0), itermax(30000), norms(0), info(1) {
  }

  /*
   * \brief finds a fixpoint starting on the initialGuess values
   * \param intialGuess starting value for the fixpoint iteration
   * \return vector after iteration (solution or currentGuess-value)
   */
  Vec MultiDimensionalFixpointSolver::solve(const Vec & initialGuess) {
    Vec currentGuess = initialGuess.copy();
    Vec lastGuess;
    norms.clear();
    for (iter = 0; iter < itermax; iter++) {
      lastGuess = currentGuess.copy();

      currentGuess = (*function)(currentGuess);

      norms.push_back(nrm2(currentGuess - lastGuess));
      if (norms[iter] < tol) {
        info = 0;
        return currentGuess;
      }

      double test = fabs(norms[norms.size() - 2] - norms[norms.size() - 1]);
      if(test  <  tol) {
        info = 2; //slow convergence --> stop
        return currentGuess;
    }

//      if(norms[norms.size() - 2] - norms[norms.size() - 1]  < 0) {
//        info = -1; //divergence --> stop
//        return lastGuess;
//      }

    }

    info = 1; //convergence (needs to be true for all steps)
    for (size_t i = 1; i < norms.size(); i++) {
      if (norms[i - 1] <= norms[i]) {
        info = -1; //no convergence
        break;
      }
    }

    return currentGuess;
  }

} /* namespace MBSimNumerics */
