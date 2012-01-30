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

#ifndef NONLINEAR_ALGEBRA_H_
#define NONLINEAR_ALGEBRA_H_

#include "mbsim/utils/function.h"
#include "fmatvec.h"

namespace MBSim {

  enum SolverType {
    RegulaFalsiSolver, FixPointIterationSolver, NewtonMethodSolver
  };

  /*! 
   * \brief Regular Falsi for one-dimensional root-finding
   * \author Martin Foerg
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class RegulaFalsi {
    public:
      /*
       * \brief constructor
       * \param root function
       */
      RegulaFalsi(Function1<double,double> *f);

      /* GETTER / SETTER */
      int getNumberOfIterations() const { return it; };
      int getInfo() const { return info; };
      void setMaximumNumberOfIterations(int itmax_) { itmax = itmax_; }
      void setTolerance(double tol_) { tol = tol_; }
      /***************************************************/

      /**
       * \brief solve nonlinear root function
       * \param left border
       * \param right border
       */
      double solve(double a, double b);

    private:
      /**
       * \brief root function
       */
      Function1<double,double> *func;

      /**
       * \brief maximum number of iterations, actual number of iterations, information about success (0 = ok, -1 = not converged, -2 = no root) 
       */
      int itmax, it, info;

      /** 
       * \brief tolerance
       */
      double tol;
  };

  /**
   * \brief FixpointIteration for multi-dimensional fixpoint-finding
   * \author Kilian Grundl
   * \date 2011-07-13 initial algorithm
   */
  class MultiDimFixPointIteration {
    public:
      /*
       * \brief constructor
       * \param fct pointer to used fix-point-function
       */
      MultiDimFixPointIteration(Function1<fmatvec::Vec, fmatvec::Vec> *function_);

      /* GETTER / SETTER */
      /*
       * \brief returns info of iteration progress
       * info == 0 :  a solution has been found
       * info == -1:  no converge
       * info == 1:   process (seems to) converge but hasn't finished
       */
      int getInfo() {
        return info;
      }
      std::vector<double> getNorms() {
        return norms;
      }
      double getNumberOfIterations() {
        return iter;
      }
      double getNumberOfMaximalIterations() {
        return itermax;
      }
      void setNumberOfMaximalIterations(int itermax_) {
        itermax = itermax_;
      }
      double getTolerance() {
        return tol;
      }
      void setTolerance(double tol_) {
        tol = tol_;
      }
      /*******************/

      fmatvec::Vec solve(const fmatvec::Vec &initialGuess);

    private:
      /**
       * \brief fix-point function
       */
      Function1<fmatvec::Vec, fmatvec::Vec> *function;

      /**
       * \brief tolerance
       */
      double tol;

      /**
       *  \brief number of iterations
       */
      int iter;

      /**
       * \brief maximal iterations
       */
      double itermax;

      /**
       *  \brief vector of norms
       */
      std::vector<double> norms;

      /**
       *  \brief information variable about success of iteration
       */
      int info;
  };

  /*! 
   * \brief Newton method for one-dimensional root-finding
   * \author Martin Foerg
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */ 
  class NewtonMethod {
    public:
      /*
       * \brief constructor
       * \param root function
       * \param Jacobian matrix
       */
      NewtonMethod(Function1<double,double> *fct_, Function1<double,double> *jac_=0);

      /* GETTER / SETTER */
      int getNumberOfIterations() const { return iter; }
      int getInfo() const { return info; }
      void setMaximumNumberOfIterations(int itmax_) { itmax = itmax_; }
      void setMaximumDampingSteps(int kmax_) { kmax = kmax_; }
      void setTolerance(double tol_) { tol = tol_; }
      /***************************************************/

      /**
       * \brief solve nonlinear root function
       * \param initial value
       */
      double solve(const double &x);

    private:
      /**
       * \brief root function
       */
      Function1<double,double> *fct;

      /**
       * \brief Jacobian matrix
       */
      Function1<double,double> *jac;

      /** 
       * \brief maximum number of iterations, actual number of iterations, maximum number of damping steps, information about success 
       */
      int itmax, iter, kmax, info;

      /** 
       * \brief tolerance
       */
      double tol;
  };

  /*! 
   * \brief Newton method for multi-dimensional root-finding
   * \author Martin Foerg
   * \date 2009-07-10 some comments (Thorsten Schindler)
   */
  class MultiDimNewtonMethod {
    public:
      /*
       * \brief constructor
       * \param root function
       * \param Jacobian matrix
       */
      MultiDimNewtonMethod(Function1<fmatvec::Vec,fmatvec::Vec> *fct_, Function1<fmatvec::SqrMat,fmatvec::Vec> *jac_=0);

      /* GETTER / SETTER */
      int getNumberOfIterations() const { return iter; }
      int getNumberOfMaximalIterations() const { return itmax; }
      int getInfo() const { return info; }
      std::vector<double> getNorms() { return norms; }
      void setMaximumNumberOfIterations(int itmax_) { itmax = itmax_; }
      void setMaximumDampingSteps(int kmax_) { kmax = kmax_; }
      void setTolerance(double tol_) { tol = tol_; }
      /***************************************************/

      /**
       * \brief solve nonlinear root function
       * \param initial value
       */
      fmatvec::Vec solve(const fmatvec::Vec &x);

    private:
      /**
       * \brief root function
       */
      Function1<fmatvec::Vec,fmatvec::Vec> *fct;

      /**
       * \brief Jacobian matrix
       */
      Function1<fmatvec::SqrMat,fmatvec::Vec> *jac;

      /** 
       * \brief maximum number of iterations, actual number of iterations, maximum number of damping steps, information about success 
       */
      int itmax, iter, kmax, info;

      /**
       *  \brief vector of norms for each iteration step
       */
      std::vector<double> norms;

      /** 
       * \brief tolerance
       */
      double tol;
  };
  
  /*
   * \brief Lemke Algorithm that solves an LCP with w = Mz + q, w^Tz = 0 and w >= 0, z >= 0
   *
   * \todo it is linear algebra, or optimisation, but actually no nonlinear algebra
   * \todo stabilize the algorithm (examples are given in "A Numerically Robust LCP Solver for Simulating Articulated Rigid Bodies in Contact" (Katsu Yamane and  Yoshihiko Nakamura) )
   * \todo make it faster (now it is Order of 2*n^2 (=GaussJordanEliminationStep) for one step) --> n^2 should be possible)
   */

  class LemkeAlgorithm {
    public:
      LemkeAlgorithm(const bool & INFO_ = false): INFO(INFO_) {
      }

      LemkeAlgorithm(const fmatvec::SqrMat & M_, const fmatvec::Vec & q_, const bool & INFO_ = false) :
          M(M_), q(q_), INFO(INFO_) {
        assert(M_.rows() == q.size());
        assert(M_.cols() == q.size());
      }

      LemkeAlgorithm(const fmatvec::SymMat & M_, const fmatvec::Vec & q_, const bool & INFO_ = false) {
        setSystem(M_, q_);
      }

      /* GETTER / SETTER */
      /**
       * \brief return info of solution process
       */
      int getInfo() {
        return info;
      }

      /**
       * \brief set system with Matrix M and vector q
       */
      void setSystem(const fmatvec::SqrMat & M_, const fmatvec::Vec & q_) {
        assert(M_.rows() == q.size());
        assert(M_.cols() == q.size());
        M = M_;
        q = q_;
      }

      /**
       * \brief set system with Matrix M and vector q
       */
      void setSystem(const fmatvec::SymMat & M_, const fmatvec::Vec & q_) {
        M = fmatvec::SqrMat(M_.size(), M_.size(), fmatvec::NONINIT);
        for(int i = 0; i < M.size(); i++)
          for(int j = 0; j < M.size(); j++)
            M(i, j) = M_(i, j);
        q = q_;
      }
      /***************************************************/

      /**
       * \brief solve algorithm adapted from : Fast Implementation of Lemke’s Algorithm for Rigid Body Contact Simulation (John E. Lloyd)
       */
      fmatvec::Vec solve(unsigned int maxloops = 0);

      virtual ~LemkeAlgorithm() {
      }

    protected:
      int findLexicographicMinimum(const fmatvec::Mat &A, const int & pivotColIndex);
      bool LexicographicPositive(const fmatvec::Vec & v);
      void GaussJordanEliminationStep(fmatvec::Mat &A, int pivotRowIndex, int pivotColumnIndex, const std::vector<size_t> & basis);
      bool greaterZero(const fmatvec::Vec & vector);
      bool validBasis(const std::vector<size_t> & basis);

      fmatvec::SqrMat M;
      fmatvec::Vec q;

      /**
       * \brief print debug output
       */
      bool INFO;

      /**
       * \brief did the algorithm find a solution
       *
       * -1 : not successful
       *  0 : successful
       */
      int info;
  };
}

#endif /* NONLINEAR_ALGEBRA_H_ */

