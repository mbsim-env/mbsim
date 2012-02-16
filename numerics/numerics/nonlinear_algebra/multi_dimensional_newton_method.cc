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

#include <config.h>

#include "multi_dimensional_newton_method.h"

#include <numerics/utils/eps.h>

#include <iostream>

using namespace fmatvec;
using namespace std;

namespace fmatvec {
  bool operator<(const Index & i1, const Index & i2) {
      if(i1.start() < i2.start())
        return true;
      else if(i1.start() > i2.start())
        return false;
      else {
        if(i1.end() < i2.end())
          return true;
        else
          return false;
      }
  }
}

/*MultiDimensionalNewtonMethod*/
namespace MBSimNumerics {

  MultiDimensionalNewtonMethod::MultiDimensionalNewtonMethod(Function1<fmatvec::Vec, fmatvec::Vec> *function_, NewtonJacobianFunction *jacobian_, DampingFunction *damping_, CriteriaFunction *critera_) :
      function(function_), jacobian(jacobian_), damping(damping_), criteria(critera_), itermax(300), iter(0), info(1) {
  }

  Vec MultiDimensionalNewtonMethod::solve(const Vec & initialValue) {
    /*Reset for comparing*/
    criteria->clear();
    criteria->setFunction(function);
    info = (*criteria)(initialValue);
    if(info == 0)
      return initialValue;

    if(damping) {
      damping->setCriteriaFunction(criteria);
      damping->setFunction(function);
    }

    jacobian->setFunction(function);

    info = 1;
    iter = 0;
    /*End - Reset*/

    //current position in function
    Vec x = initialValue;

    //current value of function
    Vec f = (*function)(x);

    //direction of function (derivative)
    SqrMat J = (*jacobian)(x);

    //step to next position
    Vec dx = slvLU(J, f);

    //damp solution
    x -= dx;

    f = (*function)(x);

    for (iter = 0; iter < itermax; iter++) {

      //Get the information about the criteria
      info = (*criteria)(x);

      //Criteria with info = 1 means: go on  (else there might be a solution found (=0) or something else)
      if (info != 1) {
        cout << "iterations " << iter << endl;
        if(info == -1) //new solution is worse than solution before --> return solution before
          return x + dx;
        else
          return x;
      }

      //compute current value
      f = (*function)(x);

      //Compute Jacobian
      J = (*jacobian)(x);

      //get step
      dx = slvLU(J, f);

      //Damp the solution
      if(damping)
        x -=  (*damping)(x, dx) * dx;
      else
        x -= dx;

    }

    info = (*criteria)(x);
    return x;
  }

}
