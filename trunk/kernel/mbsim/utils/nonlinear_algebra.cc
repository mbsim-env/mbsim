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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/utils/eps.h"
#include <mbsim/utils/utils.h>

#include <cmath>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  RegulaFalsi::RegulaFalsi(Function1<double,double> *f) : func(f), itmax(10000), tol(1e-10) {}

  double RegulaFalsi::solve(double a, double b) {

    double u=0;
    double fa, fb, fu;

    for(it=0; it<itmax; it++) {
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

  MultiDimFixPointIteration::MultiDimFixPointIteration(Function1<Vec, Vec> *function_) :
      function(function_), tol(1e-10), iter(0), itermax(30000), norms(0), info(1) {
  }

  /*
   * \brief finds a fixpoint starting on the initialGuess values
   * \param intialGuess starting value for the fixpoint iteration
   * \return vector after iteration (solution or currentGuess-value)
   */
  Vec MultiDimFixPointIteration::solve(const Vec & initialGuess) {
    Vec currentGuess = initialGuess.copy();
    Vec lastGuess;
    norms.clear();
    for (iter = 0; iter < itermax; iter++) {
      lastGuess = currentGuess.copy();

      currentGuess = (*function)(currentGuess);

      norms.push_back(nrmInf(currentGuess - lastGuess));
      if (norms[iter] < tol) {
        info = 0;
        return currentGuess;
      }

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

  NewtonMethod::NewtonMethod(Function1<double,double> *fct_, Function1<double,double> *jac_) : fct(fct_), jac(jac_), itmax(300), iter(0), kmax(100), tol(1e-10) {}

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
        double delta = epsroot();
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

  MultiDimNewtonMethod::MultiDimNewtonMethod(Function1<Vec,Vec> *fct_, Function1<SqrMat,Vec> *jac_) : fct(fct_), jac(jac_), itmax(300), iter(0), kmax(100), norms(0), tol(1e-10) {}

  Vec MultiDimNewtonMethod::solve(const Vec &x0) {

    iter=0;
    Vec x, xold;
    x = x0;

    Vec f = (*fct)(x);
    norms.clear();
    norms.push_back(nrmInf(f));
    SqrMat J;
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
        Vec f2;

        for(int j=0; j<x.size(); j++) {
          xj = x(j);

          dx = (epsroot() * 0.5);
          do {
            dx += dx;
          } while (fabs(xj + dx - x(j))<epsroot());

          x(j)+=dx;
          f2 = (*fct)(x);
          x(j)=xj;
          J.col(j) = (f2-f)/dx;
        }
      }

      Vec dx = slvLU(J,f);

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
    }

    info = 1; //convergence (needs to be true for all steps)
    for (size_t i = 1; i < norms.size(); i++) {
      if (norms[i - 1] <= norms[i]) { //divergence
        info = -1;
        break;
      }
    }

    return x;
  }

  Vec LemkeAlgorithm::solve() {
    /*REMARK:
     * This algorithm is taken from:
     * Das lineare Komplementaritätsproblem: Eine Einführung  (Uwe Schäfer) pages 7 ff
     */

    size_t dim = q.size();

    Vec solutionVector(2 * dim, INIT, 0.);

    Mat A(dim, 2 * dim + 2, NONINIT);
    A(0, 0, dim - 1, dim - 1) = Mat(dim, dim, EYE);
    A(0, dim, dim - 1, 2 * dim - 1) = (-M).copy();
    A(0, 2 * dim, dim - 1, 2 * dim) = Vec(dim, INIT, -1.);
    A(0, 2 * dim + 1, dim - 1, 2 * dim + 1) = q.copy();

    Vec q_;
    q_.resize() >> A(0, 2 * dim + 1, dim - 1, 2 * dim + 1);

    vector<size_t> basis;
    //At first, all w-values are in the basis
    for (size_t i = 0; i < dim; i++)
      basis.push_back(i);

    int pivotRowIndex = minIndex(q);     // first row is that with lowest q-value
    int pivotColIndex = 2 * dim;         //first col is that of z0

    if (INFO) {
      cout << "A: " << A << endl;
      cout << "pivotRowIndex " << pivotRowIndex << endl;
      cout << "pivotColIndex " << pivotColIndex << endl;
      cout << "Basis: ";
      for (size_t i = 0; i < basis.size(); i++)
        cout << basis[i] << " ";
      cout << endl;
    }

    if(!greaterZero(q_)) {
      int i=0;
      for(; i < pow(2,dim); i++ ) {

        if (i>0 and validBasis(basis))
          break;

        GaussJordanEliminationStep(A, pivotRowIndex, pivotColIndex, basis);

        if (INFO) {
          cout << "A: " << A << endl;
          cout << "pivotRowIndex " << pivotRowIndex << endl;
          cout << "pivotColIndex " << pivotColIndex << endl;
          cout << "Basis: ";
          for (size_t i = 0; i < basis.size(); i++)
            cout << basis[i] << " ";
          cout << endl;
        }

        int pivotColIndexOld = pivotColIndex;

        /*find new column index */
        if (basis[pivotRowIndex] < dim) //if a w-value left the basis get in the correspondent z-value
          pivotColIndex = basis[pivotRowIndex] + dim;
        else
          //else do it the other way round and get in the corresponding w-value
          pivotColIndex = basis[pivotRowIndex] - dim;

        /*the column becomes part of the basis*/
        basis[pivotRowIndex] = pivotColIndexOld;

        pivotRowIndex = findLexicographicMinimum(A, pivotColIndex);

      }
      if(INFO) {
        cout << "Number of loops: " << i << endl;
        cout << "Number of maximal loops: " << pow(2,dim) << endl;
      }

      if(!validBasis(basis))
        throw MBSimError("LemkeAlgorithm::solve: Solution process ended with ray termination!");
    }

    if (INFO) {
      cout << "A: " << A << endl;
      cout << "pivotRowIndex " << pivotRowIndex << endl;
      cout << "pivotColIndex " << pivotColIndex << endl;
    }

    for (size_t i = 0; i < basis.size(); i++)
      solutionVector(basis[i]) = q_(i);

    return solutionVector;
  }

  int LemkeAlgorithm::findLexicographicMinimum(const fmatvec::Mat &A, const int & pivotColIndex) {
    int RowIndex = 0;
    size_t dim = A.rows();
    vector<Vec> Rows;
    for (size_t row = 0; row < dim; row++) {
      Rows.push_back(Vec(dim + 1, INIT, 0.));
      double a = A(row, pivotColIndex);
      if (a > 0) {
        Rows[row](0) = A(row, 2 * dim + 1) / a;
        Rows[row](1) = A(row, 2 * dim) / a;
        for (size_t j = 2; j < dim + 1; j++)
          Rows[row](j) = A(row, j - 1) / a;

        if (INFO) {
          cout << "Rows(" << row << ") = " << Rows[row] << endl;
        }
      }
    }

    for (size_t i = 0; i < Rows.size(); i++) {
      if (nrm2(Rows[i]) > 0.) {

        size_t j = 0;
        for (; j < Rows.size(); j++) {
          if(i != j)
            if(nrm2(Rows[j]) > 0.)
              if (! LexicographicPositive(Rows[j] - Rows[i]))
                break;
        }

        if (j == Rows.size()) {
          RowIndex += i;
          break;
        }
      }
    }

    return RowIndex;
  }

  bool LemkeAlgorithm::LexicographicPositive(const fmatvec::Vec & v) {
    int i = 0;
    if (INFO)
      cout << "v " << v << endl;

    while (fabs(v(i)) < macheps() and i < v.size())
      i++;
    if (v(i) > 0)
      return true;

    return false;
  }

  void LemkeAlgorithm::GaussJordanEliminationStep(Mat &A, int pivotRowIndex, int pivotColumnIndex, const vector<size_t> & basis) {
    double a = -1 / A(pivotRowIndex, pivotColumnIndex);
    for (int i = 0; i < A.rows(); i++)
      if (i != pivotRowIndex)
        for (int j = 0; j < A.cols(); j++)
          if (j != pivotColumnIndex)
            A(i, j) += A(pivotRowIndex, j) * A(i, pivotColumnIndex) * a;

    for (int i = 0; i < A.cols(); i++) {
      A(pivotRowIndex, i) *= -a;
    }

    for (int i = 0; i < A.rows(); i++)
      if (i != pivotRowIndex)
        A(i, pivotColumnIndex) = 0;
  }

  bool LemkeAlgorithm::greaterZero(const Vec & vector) {
    bool isGreater = true;
    for (int i = 0; i < vector.size(); i++) {
      if (vector(i) < 0) {
        isGreater = false;
        break;
      }
    }

    return isGreater;
  }

  bool LemkeAlgorithm::validBasis(const vector<size_t> & basis) {
    bool isValid = true;
    for (size_t i = 0; i < basis.size(); i++) {
      if (basis[i] >= basis.size() * 2) { //then z0 is in the base
        isValid = false;
        break;
      }
    }

    return isValid;
  }

}

