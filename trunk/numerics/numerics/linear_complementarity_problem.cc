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

#include "linear_complementarity_problem.h"

#include <numerics/nonsmooth_algebra.h>

using namespace std;
using namespace fmatvec;

//TODO: get into fmatvec
namespace fmatvec {
  SqrMat Sym2Sqr(const SymMat  & M_) {
    SqrMat M(M_.size(), NONINIT);
    for (int i = 0; i < M_.size(); i++)
      for (int j = i; j < M_.size(); j++) {
        M(i, j) = M(j, i) = M_(i, j);
      }
    return M;
  }
}

namespace MBSimNumerics {

  std::ostream& operator <<(std::ostream & o, const LCPSolvingStrategy &strategy) {
    switch (strategy) {
      case Standard:
        return o << "Standard";
      case Reformulated:
        return o << "Reformulated";
      case LemkeOnly:
        return o << "LemkeOnly";
      default:
        return o << "ERROR: Unknown LCP solving strategy";
    }
  }

  LinearComplementarityProblem::LinearComplementarityProblem(const SymMat & M_, const Vec & q_, const LCPSolvingStrategy & strategy_ /*= Standard*/, const JacobianType & jacobianType_ /*= LCPSpecial*/, const unsigned int & DEBUGLEVEL /*= 0*/) :
      q(q_), strategy(strategy_), mediumEigenValue(0.0), jacobianType(jacobianType_) {

    M = Sym2Sqr(M_);
  }

  LinearComplementarityProblem::~LinearComplementarityProblem() {
  }

  void LinearComplementarityProblem::setSystem(const SymMat & M_, const Vec & q_) {
    q = q_;
    M = Sym2Sqr(M_);
  }

  Vec LinearComplementarityProblem::solve(const Vec & initialSolution /*= Vec(0,NONINIT)*/) {

    /*dimension of the system*/
    int dimension = q.size();

    //flag (is system solved?)
    bool solved = false;

    //solution vector
    Vec solution(2 * dimension, NONINIT);

    Vec w;
    w >> solution(0, dimension - 1);
    Vec z;
    z >> solution(dimension, 2 * dimension - 1);

    //set different solvers
    LemkeAlgorithm LemkeSolver;
    LemkeSolver.setSystem(M, q);

    clock_t t_start = clock();

    if (DEBUGLEVEL >= 1) {
      cout << "*****" << __func__ << "*****" << endl;
      cout << "Solving-strategy is: " << strategy << endl;
      cout << "dimension: " << dimension << endl;
    }

    //NOTE: Fortran-NewtonSolver seems to work worse than the "native" NewtonSolver of the utils in mbsim
    //      MultiDimNewtonMethodFortran NewtonSolverFortran;
    //      NewtonSolverFortran.setRootFunction(&func);
    //      Vec gapLambdaFortran = NewtonSolverFortran.solve(gapLambda0);
    if (strategy != LemkeOnly) {

      if (strategy == Standard) {
        clock_t t_start_Lemke1 = clock();
        solution = LemkeSolver.solve(dimension);

        if (LemkeSolver.getInfo() == 0) {
          solved = true;

          if (DEBUGLEVEL >= 1) {
            cout << "LemkerSolver found solution in " << LemkeSolver.getSteps() << " step(s)." << endl;
            if (DEBUGLEVEL >= 2) {
              double cpuTime = double(clock() - t_start_Lemke1) / CLOCKS_PER_SEC;
              cout << "... in: " << cpuTime << "s = " << cpuTime / 3600 << "h" << endl;
              if (DEBUGLEVEL >= 4) { //Lemke-Solver Info
                cout << "solution: " << solution << endl;
                cout << "gaps       forces   " << endl;
                for (int i = 0; i < solution.size() / 2; i++) {
                  cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
                }
              }
            }
          }
        }

        else {
          if (DEBUGLEVEL >= 1) {
            cout << "No Solution for Lemke-Algorithm. Now Trying combination of Newton + FixpointIteration" << endl;
          }
        }
      } //End Lemke-Solver in standard strategy

      clock_t t_start_Iterative = clock();

      /*Get initial solution for the reformulated system to apply recursive schemes*/
      if (not solved) {
        if (initialSolution.size() == 0)
          solution = createInitialSolution(M, q);
        else {
          assert(initialSolution.size() == 2*dimension);
          solution = initialSolution;
        }
      }

      /*Try to solve the reformulated system with iterative schemes*/
      for (int loop = 0; loop < 5 and not solved; loop++) {

        /*create the Reformulate the LCP as a system of equations*/
        LCPNewtonReformulationFunction newtonFunc = LCPNewtonReformulationFunction(q, M);

        JacobianFunction * jacobian;
        if(jacobianType == LCPSpecial)
          jacobian = new LinearComplementarityJacobianFunction();
        else
          jacobian = new NumericalJacobianFunction();

        /*Newton Solver*/
        MultiDimensionalNewtonMethod NewtonSolver(&newtonFunc, jacobian);
        NewtonSolver.setMaximumNumberOfIterations((int) 1e3);

        if (DEBUGLEVEL >= 1)
          cout << "Trying Newton Solver ... " << endl;

        for (int i = 0; i < 3 and NewtonSolver.getInfo() == 1; i++) {
          solution = NewtonSolver.solve(solution);

          if (NewtonSolver.getInfo() == 1)
            if (DEBUGLEVEL >= 3) {
              cout << "Newton scheme seems to converge but has not finished" << endl;
              cout << "Starting it again .. " << endl;
            }
        }

        if (DEBUGLEVEL >= 3) {
          cout << "Info about NewtonSolver" << endl;
          cout << "nrm2(f(solution)):  " << nrm2(newtonFunc(solution)) << endl;
          if (DEBUGLEVEL >= 4) { //Newton-Solver Info
            cout << "solution: " << solution << endl;
            cout << "gaps       forces   " << endl;
            for (int i = 0; i < solution.size() / 2; i++) {
              cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
            }
          }
        }

        switch (NewtonSolver.getInfo()) {
          case 0:
            solved = true;

            if (DEBUGLEVEL >= 1) {
              cout << "Newton-Solver found solution" << endl;

              if (DEBUGLEVEL >= 2) {
                double cpuTime = double(clock() - t_start_Iterative) / CLOCKS_PER_SEC;
                cout << "... in: " << cpuTime << "s = " << cpuTime / 3600 << "h" << endl;
                if (DEBUGLEVEL >= 4) { //Newton-Solver Info
                  cout << "solution: " << solution << endl;
                  cout << "gaps       forces   " << endl;
                  for (int i = 0; i < solution.size() / 2; i++) {
                    cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
                  }
                }
              }
            }
          break;
          case -1: {
            if (DEBUGLEVEL >= 1) {
              cout << "No convergence of Newton scheme during calculation of contact forces" << endl;
            }

            /*Fixpoint Solver*/
            LCPFixpointReformulationFunction fixpointFunc = LCPFixpointReformulationFunction(q, M);
            MultiDimensionalFixpointSolver FixpointIterator(&fixpointFunc);
            FixpointIterator.setNumberOfMaximalIterations((int) 1e4);
            FixpointIterator.setTolerance(1e-4);

            if (DEBUGLEVEL >= 1)
              cout << "Trying Fixpoint Solver ... " << endl;

            for (int i = 0; i < 3 and FixpointIterator.getInfo() == 1; i++)
              solution = FixpointIterator.solve(solution);

            if (DEBUGLEVEL >= 3) {
              cout << "Info about FixpointSolver" << endl;
              cout << "nrm2(f(solution)):  " << nrm2(newtonFunc(solution)) << endl;
              if (DEBUGLEVEL >= 4) {
                cout << "solution: " << solution << endl;
                cout << "gaps       forces   " << endl;
                for (int i = 0; i < solution.size() / 2; i++) {
                  cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
                }
              }
            }

            if (FixpointIterator.getInfo() == 0) {
              solved = true;

              if (DEBUGLEVEL >= 1) {
                cout << "Fixpoint-Solver found solution" << endl;
                if (DEBUGLEVEL >= 2) {
                  double cpuTime = double(clock() - t_start_Iterative) / CLOCKS_PER_SEC;
                  cout << "... in: " << cpuTime << "s = " << cpuTime / 3600 << "h" << endl;
                  if (DEBUGLEVEL >= 4) {
                    cout << "solution: " << solution << endl;
                    cout << "gaps       forces   " << endl;
                    for (int i = 0; i < solution.size() / 2; i++) {
                      cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
                    }
                  }
                }
              }
            }

            break;
          }
//          default:
//            throw error("ERROR MaxwellContact::solveLCP(double t): No convergence during calculation of contact forces with reformulated system scheme!");
//            break;
        }
      }

      /*If no solution is found with the other algorithms use the Lemke-Algorithm with a maxmimal number of steps as fallback*/
      if (!solved) {

        if (DEBUGLEVEL >= 1) {
          cout << "No convergence during calculation of contact forces with reformulated system scheme!" << endl;
          cout << "Using Lemke Algorithm with maximal number of steps as fallback." << endl;
        }

      }

    }

    if (not solved) {

      clock_t t_start_Lemke1 = clock();
      LemkeSolver.setSystem(M, q);
      solution = LemkeSolver.solve();

      if (LemkeSolver.getInfo() == 0) {
        solved = true;
        if (DEBUGLEVEL >= 1) {
          cout << "LemkerSolver found solution (in fallback case): " << LemkeSolver.getSteps() << " step(s)." << endl;
          if (DEBUGLEVEL >= 2) {
            double cpuTime = double(clock() - t_start_Lemke1) / CLOCKS_PER_SEC;
            cout << "... in: " << cpuTime << "s = " << cpuTime / 3600 << "h" << endl;
            if (DEBUGLEVEL >= 4) { //Newton-Solver Info
              cout << "solution: " << solution << endl;
              cout << "gaps       forces   " << endl;
              for (int i = 0; i < solution.size() / 2; i++) {
                cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
              }
            }
          }
        }
      }
      else {
        throw; // MBSimError("ERROR MaxwellContact::solveLCP(): No Solution found for this LCP");
      }
    }

    if (DEBUGLEVEL >= 1) {
      double cpuTime = double(clock() - t_start) / CLOCKS_PER_SEC;
      cout << "The complete algorithm needed: " << cpuTime << "s = " << cpuTime / 3600 << "h" << endl;
    }

    return solution;
  }

  double LinearComplementarityProblem::computeMediumEigVal(const SqrMat & M) {
    /*update Material constant*/
    Vector<complex<double> > Eigvals = eigval(M);
    double eigvalSum = 0;
    for (int i = 0; i < Eigvals.size(); i++) {
      eigvalSum += Eigvals(i).real(); //TODO: handle complex values
    }
    return eigvalSum / Eigvals.size();
  }

  Vec LinearComplementarityProblem::createInitialSolution(const SymMat & M, const Vec & q, double mediumEigVal /*= 0*/) {
    return createInitialSolution(Sym2Sqr(M), q, mediumEigVal);
  }

  Vec LinearComplementarityProblem::createInitialSolution(const SqrMat & M, const Vec & q, double mediumEigVal /*= 0*/) {

    if (mediumEigVal <= 0)
      mediumEigVal = computeMediumEigVal(M);

    /*Initialize solution*/
    Vec solution(q.size() * 2, INIT, 0.);
    for (int i = 0; i < q.size(); i++) {
      if (q(i) > 0)
        solution(i) = q(i);
      else
        solution(i) = 0;
    }

    for (int i = q.size(); i < 2 * q.size(); i++) {
      if (q(i - q.size()) > 0)
        solution(i) = 0;
      else
        solution(i) = -q(i - q.size()) / mediumEigVal;
    }

    return solution;
  }

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

    if (DEBUGLEVEL > 0) {

      cout << "w is: " << w << endl;
      cout << "z is: " << z << endl;

      cout << "M is: " << M << endl;
    }

    //compute first part
    returnVec(0, NumberOfContacts - 1) = q + M * z - w;

    //loop for the prox-functions
    for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
      returnVec(NumberOfContacts + contactIterator) = proxCN(z(contactIterator) - r * w(contactIterator)) - z(contactIterator);
      if (DEBUGLEVEL > 0) {
        cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(z(contactIterator) - r * w(contactIterator)) << "- " << z(contactIterator) << endl;
        cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << z(contactIterator) << "-" << r << "*" << w(contactIterator) << endl;
      }
    }

    if (DEBUGLEVEL > 0)
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

  LinearComplementarityJacobianFunction::LinearComplementarityJacobianFunction() {}

  LinearComplementarityJacobianFunction::~LinearComplementarityJacobianFunction() {
  }

  void LinearComplementarityJacobianFunction::setFunction(Function1<Vec, Vec> * function_) {
    if(dynamic_cast<LCPNewtonReformulationFunction*>(function_)) {
      function = function_;

      //Initialize Jacobians constant parts
      int dim = 2 * static_cast<LCPNewtonReformulationFunction*>(function_)->getq().size();

      J.resize() = SqrMat(dim, INIT, 0.);


      J(0,0,dim/2-1 ,dim/2-1) = -SqrMat(dim/2, EYE);
      J(0,dim/2, dim/2-1, dim-1) = static_cast<LCPNewtonReformulationFunction*>(function_)->getM();
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
    Vec w;
    w << x(0,dim/2-1);
    Vec z;
    z << x(dim/2, dim-1);

    //only to update lower half of Jacobian matrix
    for(int i=0; i<dim/2; i++) {
      if(z(i) < r*w(i)) {
        J(dim/2+i, i) = 0;
        J(dim/2+i, dim/2+i) = -1;
      }
      else if (z(i) > r*w(i)) {
        J(dim/2+i, i) = -r;
        J(dim/2+i, dim/2+i) = 0;
      }
      else { //at the kink one has to choose a derivative value (for now use the medium value)
        J(dim/2+i, i) = -r/2;
        J(dim/2+i, dim/2+i) = -0.5;
      }

    }
  }

}
