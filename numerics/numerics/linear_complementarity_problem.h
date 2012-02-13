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

#include <numerics/multi_dimensional_newton_method.h>
#include <numerics/multi_dimensional_fixpoint_solver.h>
#include <numerics/lemke_algorithm.h>

namespace MBSimNumerics {

  enum LCPSolvingStrategy {
    Standard,      //trying to solve the LCP in a standard way (first some steps of Lemke algorithm, then trying iterative schemes in reformulated system, then trying Lemke again as fallback with all steps)
    Reformulated,  //trying to use the iterative schemes at first (as fallback use Lemke)
    LemkeOnly
  //only use LemkeAgorithm
  };

  enum JacobianType {
    Numerical,  //use only a numerical Jacobian Matrix
    LCPSpecial
  //use a analytical Jacobian Matrix almost everywhere and choose a derivation at the kink (=LinearComplementarityJacobianFunction) //TODO: find a better name
  };

  std::ostream& operator <<(std::ostream & o, const LCPSolvingStrategy &strategy);

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

      /**
       * \brief Output (information) level
       */
      unsigned int DEBUGLEVEL;
  };

  class LCPReformulationFunction : public Function1<fmatvec::Vec, fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       */
      LCPReformulationFunction(const fmatvec::Vec &q_, const fmatvec::SqrMat &M_, const double &r_ = 10, const unsigned int & DEBUGLEVEL_ = 0);

      /*
       * \brief destructor
       */
      virtual ~LCPReformulationFunction();

      /* INHERITED INTERFACE */
      /**
       * \param q: solution vector with
       *           first entries: w, last entries: z
       */
      virtual fmatvec::Vec operator()(const fmatvec::Vec &q, const void * = NULL) = 0;
      /***************************************************/

      /**GETTER / SETTER*/
      fmatvec::Vec getq(void) {
        return q;
      }
      void setq(const fmatvec::Vec & q_) {
        q = q_;
      }
      fmatvec::SqrMat getM(void) {
        return M;
      }
      void setM(const fmatvec::SqrMat & M_) {
        M = M_;
      }
      double getr(void) {
        return r;
      }
      void setr(const double & r_) {
        r = r_;
      }
      /*****************/

    protected:
      /**
       * \brief Number of possible contact points (= dimension of the LCP)
       */
      int NumberOfContacts;

      /**
       * \brief vector of all rigid body gaps
       */
      fmatvec::Vec q;

      /**
       * \brief Influence matrix for the contacts
       */
      fmatvec::SqrMat M;

      /**
       * \brief parameter for the prox-function (r>0)
       */
      double r;

      /**
       * \brief parameter to print information
       */
      unsigned int DEBUGLEVEL;
  };

  class LCPNewtonReformulationFunction : public LCPReformulationFunction {
    public:
      /**
       * \brief constructor
       * \param q_           constant vector of LCP
       * \param M_           Coupling Matrix of LCP
       * \param r_           r-factor for the project-function
       * \param DEBUGLEVEL_  print information to console?
       */
      LCPNewtonReformulationFunction(const fmatvec::Vec &q_, const fmatvec::SqrMat &M_, const double &r_ = 10, const unsigned int & DEBUGLEVEL_ = 0);

      /**
       * \brief destructor
       */
      virtual ~LCPNewtonReformulationFunction();

      /* INHERITED INTERFACE */
      fmatvec::Vec operator()(const fmatvec::Vec &q, const void * = NULL);
      /***************************************************/

    protected:

  };

  class LCPFixpointReformulationFunction : public LCPReformulationFunction {
    public:
      /**
       * \brief constructor
       * \param q_           constant vector of LCP
       * \param M_           Coupling Matrix of LCP
       * \param r_           r-factor for the project-function
       * \param DEBUGLEVEL_  print information to console?
       */
      LCPFixpointReformulationFunction(const fmatvec::Vec &q_, const fmatvec::SqrMat &M_, const double &r_ = 10, const unsigned int & DEBUGLEVEL_ = 0);

      /**
       * \brief destructor
       */
      virtual ~LCPFixpointReformulationFunction();

      /* INHERITED INTERFACE */
      fmatvec::Vec operator()(const fmatvec::Vec &q, const void * = NULL);
      /***************************************************/
  };

  class LinearComplementarityJacobianFunction : public JacobianFunction {
    public:
      /**
       * \brief constructor
       */
      LinearComplementarityJacobianFunction();

      /*
       * \brief destructor
       */
      virtual ~LinearComplementarityJacobianFunction();

      virtual fmatvec::SqrMat operator ()(const fmatvec::Vec & x, const void* = NULL);

      virtual void setFunction(Function1<fmatvec::Vec, fmatvec::Vec> * function_);

      void updateJacobian(const fmatvec::Vec & x);

    protected:
      /**
       * \brief Jacobian function (stays saved to save computation time, as some parts stay constant)
       */
      fmatvec::SqrMat J;

  };

}

#endif /*__LINEAR_COMPLEMENTARITY_PROBLEM_H_*/
