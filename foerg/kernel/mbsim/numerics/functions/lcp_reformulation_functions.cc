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

#include "lcp_reformulation_functions.h"

#include <mbsim/utils/nonsmooth_algebra.h>

#include <iostream>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  LCPReformulationFunction::LCPReformulationFunction(const fmatvec::Vec & q_, const SqrMat &M_, const double &r_, const unsigned int & DEBUGLEVEL_) :
      NumberOfContacts(q_.size()), q(q_), M(M_), r(r_), DEBUGLEVEL(DEBUGLEVEL_) {

    //dimensions have to be equal
    assert(M_.size() == NumberOfContacts);
  }

  LCPReformulationFunction::~LCPReformulationFunction() {

  }

  LCPNewtonReformulationFunction::LCPNewtonReformulationFunction(const fmatvec::Vec & q_, const SqrMat &M_, const double &r_ /*= 10*/, const unsigned int & DEBUGLEVEL_ /*= 0*/) :
      LCPReformulationFunction(q_, M_, r_, DEBUGLEVEL_) {
  }

  LCPNewtonReformulationFunction::~LCPNewtonReformulationFunction() {
  }

  Vec LCPNewtonReformulationFunction::operator ()(const Vec &currentSolution, const void *) {

    //check dimensions
    assert(currentSolution.size() == 2 * NumberOfContacts);

    Vec returnVec(2 * NumberOfContacts, INIT, 0.);

    //reference to gap and lambda
    Vec w;
    Vec z;
    w << currentSolution(0, NumberOfContacts - 1);
    z << currentSolution(NumberOfContacts, 2 * NumberOfContacts - 1);

    if (DEBUGLEVEL >= 5) {

      cout << "w is: " << w << endl;
      cout << "z is: " << z << endl;

      cout << "M is: " << M << endl;
    }

    //compute first part
    returnVec(0, NumberOfContacts - 1) = q + M * z - w;

    //loop for the prox-functions
    for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
      returnVec(NumberOfContacts + contactIterator) = proxCN(z(contactIterator) - r * w(contactIterator)) - z(contactIterator);
      if (DEBUGLEVEL >= 5) {
        cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(z(contactIterator) - r * w(contactIterator)) << "- " << z(contactIterator) << endl;
        cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << z(contactIterator) << "-" << r << "*" << w(contactIterator) << endl;
      }
    }

    if (DEBUGLEVEL >= 1)
      cout << "returnVec is: " << returnVec << endl;

    return returnVec;

  }

  LCPFixpointReformulationFunction::LCPFixpointReformulationFunction(const fmatvec::Vec & q_, const SqrMat &M_, const double &r_ /*= 10*/, const unsigned int & DEBUGLEVEL_ /*= 0*/) :
      LCPReformulationFunction(q_, M_, r_, DEBUGLEVEL_) {

    //dimensions have to be equal
    assert(M_.size() == NumberOfContacts);
  }

  LCPFixpointReformulationFunction::~LCPFixpointReformulationFunction() {
  }

  Vec LCPFixpointReformulationFunction::operator ()(const Vec &currentSolution, const void *) {

    //check dimensions
    assert(currentSolution.size() == 2 * NumberOfContacts);

    Vec returnVec(2 * NumberOfContacts, INIT, 0.);

    //reference to gap and lambda
    Vec w;
    Vec z;
    w << currentSolution(0, NumberOfContacts - 1);
    z << currentSolution(NumberOfContacts, 2 * NumberOfContacts - 1);

    if (DEBUGLEVEL > 0) {

      cout << "w is: " << w << endl;
      cout << "z is: " << z << endl;

      cout << "M is: " << M << endl;
    }

    returnVec(0, NumberOfContacts - 1) = q + M * z;
    //loop for the prox-functions
    for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
      returnVec(NumberOfContacts + contactIterator) = proxCN(z(contactIterator) - r * w(contactIterator));
      if (DEBUGLEVEL > 0) {
        cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(z(contactIterator) - r * w(contactIterator)) << "- " << z(contactIterator) << endl;
        cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << z(contactIterator) << "-" << r << "*" << w(contactIterator) << endl;
      }
    }

    if (DEBUGLEVEL > 0)
      cout << "returnVec is: " << returnVec << endl;

    return returnVec;

  }

  LinearComplementarityJacobianFunction::LinearComplementarityJacobianFunction() :
    NewtonJacobianFunction() {
  }

  LinearComplementarityJacobianFunction::~LinearComplementarityJacobianFunction() {
  }

  void LinearComplementarityJacobianFunction::setFunction(Function1<Vec, Vec> * function_) {
    if (dynamic_cast<LCPNewtonReformulationFunction*>(function_)) {
      function = function_;

      //Initialize Jacobians constant parts
      int dim = 2 * static_cast<LCPNewtonReformulationFunction*>(function_)->getq().size();

      J >> SqrMat(dim, INIT, 0.);

      J(0, 0, dim / 2 - 1, dim / 2 - 1) = -SqrMat(dim / 2, EYE);
      J(0, dim / 2, dim / 2 - 1, dim - 1) = static_cast<LCPNewtonReformulationFunction*>(function_)->getM();
    }
    else
      throw; //TODO: use error message
  }

  SqrMat LinearComplementarityJacobianFunction::operator ()(const Vec & x, const void*) {
    updateJacobian(x);

    return J;
  }

  void LinearComplementarityJacobianFunction::updateJacobian(const Vec & x) {
    int dim = J.size();
    double r = static_cast<LCPNewtonReformulationFunction*>(function)->getr();
    Vec w(0);
    w << x(0, dim / 2 - 1);
    Vec z(0);
    z << x(dim / 2, dim - 1);

    //only to update lower half of Jacobian matrix
    for (int i = 0; i < dim / 2; i++) {
      if (z(i) < r * w(i)) {
        J(dim / 2 + i, i) = 0;
        J(dim / 2 + i, dim / 2 + i) = -1;
      }
      else if (z(i) > r * w(i)) {
        J(dim / 2 + i, i) = -r;
        J(dim / 2 + i, dim / 2 + i) = 0;
      }
      else { //at the kink one has to choose a derivative value (for now use the medium value)
        J(dim / 2 + i, i) = -r / 2;
        J(dim / 2 + i, dim / 2 + i) = -0.5;
      }

    }
  }
}
