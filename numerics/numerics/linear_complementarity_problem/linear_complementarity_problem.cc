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
  SqrMat Sym2Sqr(const SymMat & M_) {
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
      case ReformulatedStandard:
        return o << "ReformulatedStandard";
      case ReformulatedFixpointOnly:
        return o <<"ReformulatedFixpointOnly";
      case ReformulatedNewtonOnly:
        return o << "ReformulatedNewtonOnly";
      case LemkeOnly:
        return o << "LemkeOnly";
      default:
        return o << "ERROR: Unknown LCP solving strategy";
    }
  }

  std::ostream& operator <<(std::ostream & o, const JacobianType &jacobianType) {
    switch (jacobianType) {
      case Numerical:
        return o << "Numerical";
      case LCPSpecial:
        return o << "LCPSpecial";
      default:
        return o << "ERROR: Unknown jacobian type";
    }
  }

  LinearComplementarityProblem::LinearComplementarityProblem(const SymMat & M_, const Vec & q_, const LCPSolvingStrategy & strategy_ /*= Standard*/, const JacobianType & jacobianType_ /*= LCPSpecial*/, const unsigned int & DEBUGLEVEL /*= 0*/) :
      strategy(strategy_), mediumEigenValue(0.0), jacobianType(jacobianType_) {

    //set properties

    newtonSolver = new MultiDimensionalNewtonMethod();

    fixpointSolver = new MultiDimensionalFixpointSolver();

    setSystem(M_, q_);
  }

  LinearComplementarityProblem::~LinearComplementarityProblem() {
  }

  void LinearComplementarityProblem::setSystem(const SymMat & M_, const Vec & q_) {
    q = q_;
    M = Sym2Sqr(M_);

    /*set different solvers*/
    //Lemke
    lemkeSolver.setSystem(M, q);

    //Newton
    if (jacobianType == LCPSpecial)
      jacobianFunction = new LinearComplementarityJacobianFunction();
    else
      jacobianFunction = new NumericalNewtonJacobianFunction();

    newtonSolver->setJacobianFunction(jacobianFunction);

    newtonFunction = new LCPNewtonReformulationFunction();
    newtonFunction->setSystem(M, q);
    newtonSolver->setFunction(newtonFunction);

    //Fixpoint
    fixpointFunction = new LCPFixpointReformulationFunction();
    fixpointFunction->setSystem(M, q);
    fixpointSolver->setFunction(fixpointFunction);

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

    clock_t t_start = clock();

    if (DEBUGLEVEL >= 1) {
      cout << "*****" << __func__ << "*****" << endl;
      cout << "Solving-strategy is: " << strategy << endl;
      cout << "Jacobian Type is: " << jacobianType << endl;
      cout << "dimension: " << dimension << endl;
    }

    //NOTE: Fortran-NewtonSolver seems to work worse than the "native" NewtonSolver of the utils in mbsim
    //      MultiDimNewtonMethodFortran NewtonSolverFortran;
    //      NewtonSolverFortran.setRootFunction(&func);
    //      Vec gapLambdaFortran = NewtonSolverFortran.solve(gapLambda0);
    if (strategy != LemkeOnly) {
      if (strategy == Standard) {
        clock_t t_start_Lemke1 = clock();
        solution = lemkeSolver.solve(dimension);

        if (lemkeSolver.getInfo() == 0) {
          solved = true;

          if (DEBUGLEVEL >= 1) {
            cout << "LemkerSolver found solution in " << lemkeSolver.getSteps() << " step(s)." << endl;
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

      //clock_t t_start_Iterative = clock();

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
        if(strategy == ReformulatedNewtonOnly) {
          useNewton(solution, solved);
        }
        else if (strategy == ReformulatedFixpointOnly) {
          useFixpoint(solution, solved);
        }
        else {
          useNewton(solution, solved);

          switch (newtonSolver->getInfo()) {
            case 0:

              break;
            case -1: {
              useFixpoint(solution, solved);
              break;
            }
          }
        }
      } /*End reformulated system*/

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
      lemkeSolver.setSystem(M, q);
      solution = lemkeSolver.solve();

      if (lemkeSolver.getInfo() == 0) {
        solved = true;
        if (DEBUGLEVEL >= 1) {
          cout << "LemkerSolver found solution (in fallback case): " << lemkeSolver.getSteps() << " step(s)." << endl;
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

  void LinearComplementarityProblem::useNewton(Vec & solution, bool & solved) {
    if (DEBUGLEVEL >= 1)
      cout << "Trying Newton Solver ... " << endl;

    int i = 0;
    do {
      solution = newtonSolver->solve(solution);

      if (newtonSolver->getInfo() == 1)
        if (DEBUGLEVEL >= 2) {
          cout << "Newton scheme seems to converge but has not finished --> Going on ..." << endl;
        }
    } while (i++ < 3 and newtonSolver->getInfo() == 1);

    if (DEBUGLEVEL >= 1) {
      cout << "Iterations = " << newtonSolver->getNumberOfIterations() << endl;
      if (DEBUGLEVEL >= 3) {
        cout << "Info about NewtonSolver" << endl;
        cout << "nrm2(f(solution)):  " << nrm2((*newtonFunction)(solution)) << endl;
        if (DEBUGLEVEL >= 4) { //Newton-Solver Info
          cout << "solution: " << solution << endl;
          cout << "gaps       forces   " << endl;
          for (int i = 0; i < solution.size() / 2; i++) {
            cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
          }
        }
      }
    }

    if(newtonSolver->getInfo() == 0) {
      solved = true;

      if (DEBUGLEVEL >= 1) {
        cout << "Newton-Solver found solution" << endl;

        if (DEBUGLEVEL >= 2) {
          //double cpuTime = double(clock() - t_start_Iterative) / CLOCKS_PER_SEC;
          //cout << "... in: " << cpuTime << "s = " << cpuTime / 3600 << "h" << endl;
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
  }

  void LinearComplementarityProblem::useFixpoint(Vec & solution, bool & solved) {
    if (DEBUGLEVEL >= 1)
      cout << "Trying Fixpoint Solver ... " << endl;

    int i = 0;
    do {
      solution = fixpointSolver->solve(solution);

      if (fixpointSolver->getInfo() == 1)
        if (DEBUGLEVEL >= 2) {
          cout << "Fixpoint scheme seems to converge but has not finished --> Going on ..." << endl;
        }
    } while (i++ < 3 and fixpointSolver->getInfo() == 1);

    if (DEBUGLEVEL >= 1) {
      cout << "Iterations = " << fixpointSolver->getNumberOfIterations() << endl;
      if (DEBUGLEVEL >= 3) {
        cout << "Info about FixpointSolver" << endl;
        cout << "nrm2(f(solution)):  " << nrm2((*newtonFunction)(solution)) << endl;
        if (DEBUGLEVEL >= 4) {
          cout << "solution: " << solution << endl;
          cout << "gaps       forces   " << endl;
          for (int i = 0; i < solution.size() / 2; i++) {
            cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
          }
        }
      }
    }

    if (fixpointSolver->getInfo() == 0) {
      solved = true;

      if (DEBUGLEVEL >= 1) {
        cout << "Fixpoint-Solver found solution" << endl;
        if (DEBUGLEVEL >= 2) {
          //double cpuTime = double(clock() - t_start_Iterative) / CLOCKS_PER_SEC;
          //cout << "... in: " << cpuTime << "s = " << cpuTime / 3600 << "h" << endl;
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
  }

}
