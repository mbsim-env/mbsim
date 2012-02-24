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

#include <iostream>

#include "lemke_algorithm.h"

#include <numerics/utils/eps.h>

using namespace fmatvec;
using namespace std;

namespace MBSimNumerics {

  Vec LemkeAlgorithm::solve(unsigned int maxloops /* = 0*/) {
    /*REMARK:
     * This algorithm is taken from:
     * Das lineare Komplementaritätsproblem: Eine Einführung  (Uwe Schäfer) pages 7 ff
     */

    if(DEBUGLEVEL >= 1) {
      cout << "******" << __FUNCTION__ << "*******" << endl;
    }

    steps = 0;

    size_t dim = q.size();

    if(DEBUGLEVEL >= 1) {
      cout << "Dimension = " << dim << endl;
    }

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

    size_t pivotRowIndex = minIndex(q_);     // first row is that with lowest q-value
    size_t z0Row = pivotRowIndex;           // remember the col of z0 for ending algorithm afterwards
    size_t pivotColIndex = 2 * dim;         // first col is that of z0

    if (DEBUGLEVEL >= 3) {
      cout << "A: " << A << endl;
      cout << "pivotRowIndex " << pivotRowIndex << endl;
      cout << "pivotColIndex " << pivotColIndex << endl;
      cout << "Basis: ";
      for (size_t i = 0; i < basis.size(); i++)
        cout << basis[i] << " ";
      cout << endl;
    }

    if(!greaterZero(q_)) {

      if (maxloops == 0 )
        maxloops = (1<<dim); // =pow(2, dim) but faster

      /*start looping*/
      for(steps = 0; steps < maxloops; steps++) {

        GaussJordanEliminationStep(A, pivotRowIndex, pivotColIndex, basis);

        if (DEBUGLEVEL >= 3) {
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

        if(z0Row == pivotRowIndex) { //if z0 leaves the basis the solution is found --> one last elimination step is necessary
          GaussJordanEliminationStep(A, pivotRowIndex, pivotColIndex, basis);
          basis[pivotRowIndex] = pivotColIndex; //update basis
          break;
      }

      }
      if(DEBUGLEVEL >= 1) {
        cout << "Number of loops: " << steps << endl;
        cout << "Number of maximal loops: " << maxloops << endl;
      }

      if(!validBasis(basis)) {
        info = -1;
        if(DEBUGLEVEL >= 1)
          cout << "Lemke-Algorithm ended with Ray-Termination (no valid solution)." << endl;

        return solutionVector;
      }

    }

    if (DEBUGLEVEL >= 2) {
      cout << "A: " << A << endl;
      cout << "pivotRowIndex " << pivotRowIndex << endl;
      cout << "pivotColIndex " << pivotColIndex << endl;
    }

    for (size_t i = 0; i < basis.size(); i++)
      solutionVector(basis[i]) = q_(i);

    info = 0;

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

        if (DEBUGLEVEL) {
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
    if (DEBUGLEVEL)
      cout << "v " << v << endl;

    while(i < v.size()-1 and fabs(v(i)) < macheps())
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


} /* namespace MBSimNumerics */
