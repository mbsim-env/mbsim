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

#include <mbsim/utils/eps.h>

#include <iostream>

using namespace fmatvec;
using namespace std;
using namespace std::placeholders;

namespace fmatvec {
  bool operator<(const RangeV & i1, const RangeV & i2) {
    if (i1.start() < i2.start())
      return true;
    else if (i1.start() > i2.start())
      return false;
    else {
      if (i1.end() < i2.end())
        return true;
      else
        return false;
    }
  }
}

/*MultiDimensionalNewtonMethod*/
namespace MBSim {

  inline Vec fslvLUMDNW(const SqrMat &a, const Vec& b, int & info) {
    return slvLU(a, b, info); //TODO: unify with nonlinear algebra
  }

  inline Vec fslvLSMDNW(const SqrMat &a, const Vec& b, int & info) {
    return slvLS(a, b); //TODO: unify with nonlinear algebra
  }

  MultiDimensionalNewtonMethod::MultiDimensionalNewtonMethod() :
      function(0), jacobian(0), damping(0), criteria(0), itermax(300), iter(0), info(1), linAlg(0), jacobianUpdateFreq(1) {
  }

  Vec MultiDimensionalNewtonMethod::solve(const Vec & initialValue) {
    std::function<Vec(const SqrMat&, const Vec&, int&)> slv;

    info = 1;
    iter = 0;

    if (linAlg == 0)
      slv = std::bind(fslvLUMDNW, _1, _2, _3);
    else if (linAlg == 1)
      slv = std::bind(fslvLSMDNW, _1, _2, _3);

    /*Reset for comparing*/
    criteria->clear();
    criteria->setFunction(function);
    info = (*criteria)(initialValue);
    if (info == 0)
      return initialValue;

    if (damping) {
      damping->setCriteriaFunction(criteria);
      damping->setFunction(function);
    }

    jacobian->setFunction(function);
    /*End - Reset*/

    //current position in function
    Vec x = initialValue.copy();

    //current value of function
    Vec f = (*function)(x);

    //direction of function (derivative)
    SqrMat J = (*jacobian)(x);

    //step to next position
    Vec dx = slv(J, f, info);

    //Damp the solution
    if (damping)
      x -= (*damping)(x, dx) * dx;
    else
      x -= dx;

    f = (*function)(x);

    for (iter = 1; iter < itermax; iter++) {

      //Get the information about the criteria
      info = (*criteria)(x);

      //Criteria with info = 1 means: go on  (else there might be a solution found (=0) or something else)
      if (info != 1) {
        if (info == -1) { //new solution is worse than solution before --> return solution before
          return x + dx;
        }
        else
          return x;
      }

      //compute current value
      f = (*function)(x);

      //Compute Jacobian
//      J = (*jacobian)(x);

      if ((iter % jacobianUpdateFreq) == 0) {
        J = (*jacobian)(x);
//        cout << "update Jacobian, at iter =" << iter << endl;
      }

      if (0) {
        cout << "+++++ iter = " << iter << "++++++++++++\n";
        cout << "J = " << J << "\n";
        cout << "f = " << f << "\n";
      }
      //get step
      dx = slv(J, f, info);

      //cout << "dxn[" << iter << "] = " << dx << endl;

      //Damp the solution
      if (damping)
        x -= (*damping)(x, dx) * dx;
      else
        x -= dx;

    }

    info = (*criteria)(x);
    return x;
  }

}
