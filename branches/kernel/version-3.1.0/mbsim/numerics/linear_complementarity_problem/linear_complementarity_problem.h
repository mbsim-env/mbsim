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

#ifndef NUMERICS_LINEAR_COMPLEMENTARITY_PROBLEM_H_
#define NUMERICS_LINEAR_COMPLEMENTARITY_PROBLEM_H_

#include <fmatvec.h>

#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_fixpoint_solver.h>
#include <mbsim/numerics/linear_complementarity_problem/lemke_algorithm.h>

#include <mbsim/numerics/functions/lcp_reformulation_functions.h>

namespace MBSim {

  enum LCPSolvingStrategy {
    Standard,      //trying to solve the LCP in a standard way (first some steps of Lemke algorithm, then trying iterative schemes in reformulated system, then trying Lemke again as fallback with all steps)
    ReformulatedStandard,  //trying to use the iterative schemes at first (as fallback use Lemke)
    ReformulatedNewtonOnly,
    ReformulatedFixpointOnly,
    LemkeOnly
    //only use LemkeAgorithm
  };

  enum JacobianType {
    Numerical,  //use only a numerical Jacobian Matrix
    LCPSpecial
    //use a analytical Jacobian Matrix almost everywhere and choose a derivation at the kink (=LinearComplementarityJacobianFunction) //TODO: find a better name
  };

  enum CriteriaType {
    Global,
    Local
  };
  std::ostream& operator <<(std::ostream & o, const LCPSolvingStrategy &strategy);

  std::ostream& operator <<(std::ostream & o, const JacobianType &jacobianType);


  /**
   * \brief class to solve a linear complementarity problem
   *
   * \todo: find a better concept for the properties of the used solving algorithms (maybe single "option" - classes)
   */
  class LinearComplementarityProblem {
    public:
      /**
       * \brief solves a linear complementarity problem (w = M z + q)
       * \param M                   linear coupling matrix of the LCP
       * \param q                   constant vector of the LCP
       * \param LCPSolvingStrategy  algorithm strategy to solve the LCP
       * \param mediumEigVal        parameter for finding the start solution for the reformulated system
       * \param DEBUGLEVEL  Define output (information) level
       */
      LinearComplementarityProblem(const fmatvec::SymMat & M_, const fmatvec::Vec & q_, const LCPSolvingStrategy & strategy_ = Standard, const JacobianType & jacobianType_ = LCPSpecial, const unsigned int & DEBUGLEVEL = 0);

      virtual ~LinearComplementarityProblem();

      /*GETTER / SETTER*/
      void setSystem(const fmatvec::SymMat & M_, const fmatvec::Vec & q_);
      void setStrategy(const LCPSolvingStrategy & strategy_) {
        strategy = strategy_;
      }
      void setJacobianType(const JacobianType & jacobianType_) {
        jacobianType = jacobianType_;
      }
      void setNewtonCriteriaFunction(CriteriaFunction<fmatvec::Ref, double> * criteriaFunction_) {
        newtonSolver->setCriteriaFunction(criteriaFunction_);
      }
      void setFixpointCriteriaFunction(CriteriaFunction<fmatvec::Ref, double> * criteriaFunction_) {
        fixpointSolver->setCriteriaFunction(criteriaFunction_);
      }
      void setDebugLevel(const unsigned int & DEBUGLEVEL_) {
        DEBUGLEVEL = DEBUGLEVEL_;
      }
      /****************/

      fmatvec::Vec solve(const fmatvec::Vec & initialSolution = fmatvec::Vec(0, fmatvec::NONINIT));

      /**
       * \brief compute the medium eigenvalue of a matrix (for guessing a initial solution for iterative schemes)
       */
      static double computeMediumEigVal(const fmatvec::SqrMat & M);

      /**
       * \brief creates an initial solution for the iterative schemes that use a reformulated system
       */
      static fmatvec::Vec createInitialSolution(const fmatvec::SymMat & M, const fmatvec::Vec & q, double mediumEigVal = 0);

      /**
       * \brief creates an initial solution for the iterative schemes that use a reformulated system
       */
      static fmatvec::Vec createInitialSolution(const fmatvec::SqrMat & M, const fmatvec::Vec & q, double mediumEigVal = 0);

    protected:
      /*!
       * \brief change the incoming solution vector using the fixpoint scheme
       */
      void useNewton(fmatvec::Vec & solution, bool & solved);

      /*!
       * \brief change the incoming solution vector using the fixpoint scheme
       */
      void useFixpoint(fmatvec::Vec & solution, bool & solved);

      /**
       * \brief linear coupling matrix of the LCP
       */
      fmatvec::SqrMat M;

      /**
       * \brief constant vector of the LCP
       */
      fmatvec::Vec q;

      /**
       * \brief algorithm strategy to solve the LCP
       */
      LCPSolvingStrategy strategy;

      /**
       * \brief parameter for finding the start solution for the reformulated system
       */
      double mediumEigenValue;

      /**
       * \brief jacobian function for the solving with thew newton algorithm in the reformulated case
       */
      JacobianType jacobianType;

      /*!
       * \brief LemkeSolver for the direct solution of the LCP
       */
      LemkeAlgorithm lemkeSolver;

      /*!
       * \brief NewtonSolver for reformulated system
       */
      MultiDimensionalNewtonMethod<fmatvec::Ref, double> * newtonSolver;

      /*!
       * \brief reformulated LCP suited for a Newton Solver
       */
      LCPNewtonReformulationFunction * newtonFunction;

      /*!
       * \brief Jacobian Function for the reformulated LCP
       */
      NewtonJacobianFunction<fmatvec::Ref, double> * jacobianFunction;

      /*!
       * \brief criteria function for Newton solver
       */
      CriteriaFunction<fmatvec::Ref, double> * criteriaNewton;

      /*!
       * \brief FixpointSolver for reformulated system
       */
      MultiDimensionalFixpointSolver<fmatvec::Ref, double> * fixpointSolver;

      /*!
       * \brief reformulated LCP suited for a FixpointSolver
       */
      LCPFixpointReformulationFunction * fixpointFunction;

      /*!
       * \brief criteria function for Fixedpoint solver
       */
      CriteriaFunction<fmatvec::Ref, double> * criteriaFixedpoint;

      /**
       * \brief Output (information) level
       */
      unsigned int DEBUGLEVEL;
  };
}

#endif /*__LINEAR_COMPLEMENTARITY_PROBLEM_H_*/
