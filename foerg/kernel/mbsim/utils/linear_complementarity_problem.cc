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

#include <linear_complementarity_problem.h>

#include <time.h>

#include <mbsim/utils/nonsmooth_algebra.h>

using namespace std;
using namespace fmatvec;

namespace MBSim {


  Vec solveLCP(const SymMat & M, const Vec & q, const LCPSolvingStrategy & strategy /* = Standard*/, double mediumEigVal /*= 0*/, const unsigned int & DEBUGLEVEL /* = 0*/) {

    bool converged = false;

    if(mediumEigVal <= 0)
      mediumEigVal = computeMediumEigVal(M);

    clock_t t_start = clock();
    /*dimension of the system*/
    size_t dimension = q.size();

    if (DEBUGLEVEL >= 1) {
      cout << "*****" << __func__ << "*****" << endl;
      if (DEBUGLEVEL >= 2) {
        cout << "Systemdimension is: " << dimension << endl;
      }
    }

    /*Initialize gapLamba*/
    Vec solution0(dimension * 2, INIT, 0.);
    for (size_t i = 0; i < dimension; i++) {
      if (q(i) > 0)
        solution0(i) = q(i);
      else
        solution0(i) = 0;
    }

    for (size_t i = dimension; i < 2 * dimension; i++) {
      if (q(i - dimension) > 0)
        solution0(i) = 0;
      else
        solution0(i) = -q(i - dimension) / mediumEigVal;
    }

    /*create vector for solution*/
    Vec solution(solution0.copy());

    //TODO_grundl build Jacobian, if it is possible at all --> if it is computed numerically it gets more unstable?

    //NOTE: Fortran-NewtonSolver seems to work worse than the "native" NewtonSolver of the utils in mbsim
    //      MultiDimNewtonMethodFortran NewtonSolverFortran;
    //      NewtonSolverFortran.setRootFunction(&func);
    //      Vec gapLambdaFortran = NewtonSolverFortran.solve(gapLambda0);

    /* solve the LCP */

    //use Lemke-Solver for small systems
    LemkeAlgorithm LemkeSolver;
    LemkeSolver.setSystem(M, q);

    clock_t t_start_Lemke1 = clock();
    solution = LemkeSolver.solve(100); //TODO find better value as 100

    if (LemkeSolver.getInfo() == 0) {
      converged = true;
      if (DEBUGLEVEL >= 1) {
        cout << "LemkerSolver found solution" << endl;
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
      solution = solution0;
      if (DEBUGLEVEL >= 1) {
        cout << "No Solution for Lemke-Algorithm. Now Trying combination of Newton + FixpointIteration" << endl;
      }
    }

    Vec z;
    z >> solution(dimension, 2 * dimension - 1);

    clock_t t_start_Iterative = clock();

    /*create the Reformulate the LCP as a system of equations*/
    LCPReformulationFunction func = LCPReformulationFunction(q, M);

    while (!converged) {

      /*Newton Solver*/
      MultiDimNewtonMethod NewtonSolver(&func);
      NewtonSolver.setMaximumNumberOfIterations((int) 1e4);

      func.setSolverType(NewtonMethodSolver);    //Define different solvers

      while(NewtonSolver.getInfo() == 1) {
        solution = NewtonSolver.solve(solution);

        if(NewtonSolver.getInfo() == 1)
          if (DEBUGLEVEL >= 3) {
            cout << "Newton scheme seems to converge but has'nt finished" << endl;
            cout << "Starting it again .. " << endl;
            vector<double> norms = NewtonSolver.getNorms();
            cout << "Current Norm is: " << norms[norms.size() - 1] << endl;
          }
      }

      if (DEBUGLEVEL >= 3) {
        cout << "Info about NewtonSolver" << endl;
        cout << "nrm2(f(solution)):  " << nrm2(func(solution)) << endl;
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
          converged = true;
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
        case -1:
        {
          if (DEBUGLEVEL >= 3) {
            cout << "MaxwellContact::solveLCP(double t): No convergence of Newton scheme during calculation of contact forces" << endl;
            cout << "Trying a Fixpoint-solver (at least for a good starting value for the Newton scheme) ..." << endl;
          }

          /*Fixpoint Solver*/
          MultiDimFixPointIteration FixpointIterator(&func);
          FixpointIterator.setNumberOfMaximalIterations((int) 1e6);
          FixpointIterator.setTolerance(1e-4);

          func.setSolverType(FixPointIterationSolver);

          while (FixpointIterator.getInfo() == 1)
            solution = FixpointIterator.solve(solution);

          if (DEBUGLEVEL >= 3) {
            cout << "Info about FixpointSolver" << endl;
            func.setSolverType(NewtonMethodSolver);
            cout << "nrm2(f(solution)):  " << nrm2(func(solution)) << endl;
            if (DEBUGLEVEL >= 4) {
              cout << "solution: " << solution << endl;
              cout << "gaps       forces   " << endl;
              for (int i = 0; i < solution.size() / 2; i++) {
                cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
              }
            }
            func.setSolverType(FixPointIterationSolver);
          }

          if (FixpointIterator.getInfo() == 0) {
            converged = true;

            if (DEBUGLEVEL >= 1) {
              cout << "Fixpoint-Iterator found solution" << endl;
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
        default:
          throw MBSimError("ERROR MaxwellContact::solveLCP(double t): No convergence during calculation of contact forces with Newton scheme!");
          break;
      }
    }

    if (!converged) {
      if (DEBUGLEVEL >= 1) {
        cout << "No convergence during calculation of contact forces with Newton scheme!" << endl;
        cout << "Now using Lemke Algorithm..." << endl;
      }

      clock_t t_start_Lemke1 = clock();
      solution = LemkeSolver.solve();

      if (LemkeSolver.getInfo() == 0) {
        converged = true;
        if (DEBUGLEVEL >= 1) {
          cout << "LemkerSolver found solution with the second try ..." << endl;
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

    }

    if (!converged) {
      throw MBSimError("ERROR MaxwellContact::solveLCP(): No Solution found for this LCP");
    }

    if (DEBUGLEVEL >= 1) {
      double cpuTime = double(clock() - t_start) / CLOCKS_PER_SEC;
      cout << "Solution found in: " << cpuTime << "s = " << cpuTime / 3600 << "h" << endl;
    }

    return solution;
  }

  double computeMediumEigVal(const SymMat & M) {
    /*update Material constant*/
    Vec Eigvals = eigval(M);
    double eigvalSum = 0;
    for (int i = 0; i < Eigvals.size(); i++) {
      eigvalSum += Eigvals(i);
    }
    return eigvalSum / Eigvals.size();
  }

  LCPReformulationFunction::LCPReformulationFunction(const fmatvec::Vec & q_, const SymMat &M_, const double &r_ /*= 10*/, const unsigned int & DEBUGLEVEL_ /*= 0*/) :
      NumberOfContacts(q_.size()), q(q_), M(M_), r(r_), solverType(NewtonMethodSolver), DEBUGLEVEL(DEBUGLEVEL_) {

    //dimensions have to be equal
    assert(M_.size() == NumberOfContacts);
  }

  LCPReformulationFunction::~LCPReformulationFunction() {
  }

  Vec LCPReformulationFunction::operator ()(const Vec &currentSolution, const void *) {

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
    if (solverType == NewtonMethodSolver) {
      returnVec(0, NumberOfContacts - 1) = q + M * z - w;

      //loop for the prox-functions
      for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
        returnVec(NumberOfContacts + contactIterator) = proxCN(z(contactIterator) - r * w(contactIterator)) - z(contactIterator);
        if (DEBUGLEVEL > 0) {
          cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(z(contactIterator) - r * w(contactIterator)) << "- " << z(contactIterator) << endl;
          cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << z(contactIterator) << "-" << r << "*" << w(contactIterator) << endl;
        }
      }
    }
    else if (solverType == FixPointIterationSolver) {
      returnVec(0, NumberOfContacts - 1) = q + M * z;
      //loop for the prox-functions
      for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
        returnVec(NumberOfContacts + contactIterator) = proxCN(z(contactIterator) - r * w(contactIterator));
        if (DEBUGLEVEL > 0) {
          cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(z(contactIterator) - r * w(contactIterator)) << "- " << z(contactIterator) << endl;
          cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << z(contactIterator) << "-" << r << "*" << w(contactIterator) << endl;
        }
      }
    }

    if (DEBUGLEVEL > 0)
      cout << "returnVec is: " << returnVec << endl;

    return returnVec;

  }

}
