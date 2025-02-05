/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/eps.h"
#include <mbsim/utils/utils.h>

#include <cmath>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  RegulaFalsi::RegulaFalsi(Function<double(double)> *f) : func(f), itmax(10000), it(0), info(-1), tol(1e-10) {}

  double RegulaFalsi::solve(double a, double b) {

    double u=0;
    double fa, fb, fu;

    for(it=0; it<=itmax; it++) {
      fa = (*func)(a);
      fb = (*func)(b);
      if(fa*fb > 0) {
        info = -2;
      }
      u = a - fa * (b-a)/(fb-fa);
      if(u<a)
        return u;
      else if(u>b)
        return u;
      fu = (*func)(u);

      if (fu*fa < 0) {
        b = u;
      } else {
        a = u;
      }
      if (fabs(fu) < tol) {
        info = 0;
        return u;
      }
    }
    info = -1;
    return u;
  }

  MultiDimFixPointIteration::MultiDimFixPointIteration(Function<Vec(Vec)> *function_) :
    function(function_), tol(1e-10), iter(0), itermax(30000), norms(0), info(1) {
    }

  /*
   * \brief finds a fixpoint starting on the initialGuess values
   * \param intialGuess starting value for the fixpoint iteration
   * \return vector after iteration (solution or currentGuess-value)
   */
  Vec MultiDimFixPointIteration::solve(const Vec & initialGuess) {
    Vec currentGuess = initialGuess;
    Vec lastGuess;
    norms.clear();
    for (iter = 0; iter <= itermax; iter++) {
      lastGuess = currentGuess;

      currentGuess = (*function)(currentGuess);

      norms.push_back(nrmInf(currentGuess - lastGuess));
      if (norms[iter] < tol) {
        info = 0;
        return currentGuess;
      }

      if(fabs(norms[norms.size() - 2] - norms[norms.size() - 1])  < macheps) {
        info = 1; //no more convergence possible --> stop
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

  NewtonMethod::NewtonMethod(Function<double(double)> *fct_, Function<double(double)> *jac_) : fct(fct_), jac(jac_), itmax(300), iter(0), kmax(100), tol(1e-10) {}

  double NewtonMethod::solve(const double &x0) {

    iter=0;
    double x = x0, xold;

    double f = (*fct)(x);
    double nrmf0 = fabs(f);
    double J;
    for (iter=1; iter <= itmax; iter++) {

      if (nrmf0 <= tol) {
        info = 0;
        return x0;
      }

      if(jac)
        J = (*jac)(x);
      else {
        double delta = sqrt(macheps*std::max(1.e-5,abs(x)));
        double xtmp = x;
        x += delta;
        double f_new = (*fct)(x);
        J = (f_new - f)/delta;
        x = xtmp;
      }
      double dx = 0;
      if(J*J > 1.e-15)
        dx = f/J;
      else {
        info = -1;
        return 0;
      }

      double nrmf = 1;
      double alpha = 1;
      xold = x;
      for (int k=0; k<kmax; k++) {
        x = xold - alpha*dx;
        f = (*fct)(x);
        nrmf = fabs(f);
        if(nrmf < nrmf0)
          break;
        alpha *= 0.5;
      }
      nrmf0 = nrmf;
      if (nrmf <= tol) {
        info = 0;
        return x;
      }
    }
    info = -1;

    return 0;
  }

  inline Vec fslvLU(const SqrMat &a, const Vec&  b) {
    return slvLU(a,b);
  }

  inline Vec fslvLS(const SqrMat &a, const Vec&  b) {
    return slvLS(a,b);
  }

  MultiDimNewtonMethod::MultiDimNewtonMethod(Function<Vec(Vec)> *fct_, Function<SqrMat(Vec)> *jac_) : fct(fct_), jac(jac_), itmax(300), iter(0), kmax(50), info(1), norms(0), tol(1e-10), linAlg(0) {}

  Vec MultiDimNewtonMethod::solve(const Vec &x0) {
    std::function<Vec(const SqrMat&,const Vec&)> slv;

    if(linAlg==0)
      slv = std::bind(fslvLU, placeholders::_1, placeholders::_2);
    else if(linAlg==1)
      slv = std::bind(fslvLS, placeholders::_1, placeholders::_2);

    iter=0;
    Vec x(x0.size(),NONINIT), xold(x0.size(),NONINIT);
    x = x0;

    Vec f = (*fct)(x);
    norms.clear();
    norms.push_back(nrmInf(f));
    SqrMat J(x0.size(),NONINIT);
    for (iter = 0; iter <= itmax; iter++) {

      if (norms[iter] <= tol) {
        info = 0;
        return x;
      }

      if(jac)
        J = (*jac)(x);
      else {
        J = SqrMat(x.size()); // initialise size
        double dx, xj;
        Vec f2(f.size(),NONINIT);

        for(int j=0; j<x.size(); j++) {
          xj = x(j);

          dx = sqrt(macheps*max(1.e-5,abs(xj)));

          x(j)+=dx;
          f2 = (*fct)(x);
          x(j)=xj;
          J.set(j, (f2-f)/dx);
        }
      }

      Vec dx = slv(J,f);

      double nrmf = 1;
      double alpha = 1;
      xold = x;
      for (int k=0; k<kmax; k++) {
        x = xold - alpha*dx;
        f = (*fct)(x);
        nrmf = nrmInf(f);
        if (nrmf < norms[iter])
          break;
        alpha *= 0.5;
      }
      norms.push_back(nrmf);
      if (nrmf <= tol) {
        info = 0;
        return x;
      }

      if (std::isnan(nrmf)) {
        info = -1;
        return xold;
      }

      if(fabs(norms[norms.size() - 2] - norms[norms.size() - 1]) < macheps) {
        info = -2; //divergence --> stop
        return x;
      }

    }

    info = 1; //convergence (needs to be true for all steps)
    for (size_t i = 1; i < norms.size(); i++) {
      if (norms[i - 1] <= norms[i] or std::isnan(norms[i])) { //divergence
        info = -3;
        break;
      }
    }

    return x;
  }

}

