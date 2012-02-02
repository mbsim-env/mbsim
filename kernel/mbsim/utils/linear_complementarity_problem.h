/* Copyright (C) 2004-2012  MBSim Development Team

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
 * Contact:
 *   martin.o.foerg@googlemail.com
 *
 */
#ifndef LINEAR_COMPLEMENTARITY_PROBLEM_H_
#define LINEAR_COMPLEMENTARITY_PROBLEM_H_

#include <fmatvec.h>

#include <mbsim/utils/nonlinear_algebra.h>

namespace MBSim {

  enum LCPSolvingStrategy {
    Standard
  };


  /**
   * \brief solves a linear complementarity problem (w = M z + q)
   * \param M                   linear coupling matrix of the LCP
   * \param q                   constant vector of the LCP
   * \param LCPSolvingStrategy  algorithm strategy to solve the LCP
   * \param mediumEigVal        parameter for finding the start solution for the reformulated system
   */
  fmatvec::Vec solveLCP(const fmatvec::SymMat & M, const fmatvec::Vec & q, const LCPSolvingStrategy & strategy = Standard, double mediumEigVal = 0.0, const unsigned int & DEBUGLEVEL = 0);

  /**
   * \brief update the matConst variable
   */
  double computeMediumEigVal(const fmatvec::SymMat & M);


  class LCPReformulationFunction : public MBSim::Function1<fmatvec::Vec, fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       * \param q_           constant vector of LCP
       * \param M_           Coupling Matrix of LCP
       * \param r_           r-factor for the project-function
       * \param DEBUGLEVEL_  print information to console?
       */
      LCPReformulationFunction(const fmatvec::Vec &q_, const fmatvec::SymMat &M_, const double &r_ = 10, const unsigned int & DEBUGLEVEL_ = 0);

      /**
       * \brief destructor
       */
      virtual ~LCPReformulationFunction();

      /* INHERITED INTERFACE */
      /**
       * \param q: solution vector with
       *           first entries: w, last entries: z
       */
      fmatvec::Vec operator()(const fmatvec::Vec &q, const void * = NULL);
      /***************************************************/

      /* GETTER / SETTER*/
      SolverType getSolverType() {
        return solverType;
      }
      void setSolverType(const SolverType &solverType_) {
        solverType = solverType_;
      }
      /******************/

    private:

      /**
       * \brief Number of possible contact points (= dimension of the MFL)
       */
      int NumberOfContacts;

      /**
       * \brief vector of all rigid body gaps
       */
      fmatvec::Vec q;

      /**
       * \brief Influence matrix for the contacts
       */
      fmatvec::SymMat M;

      /**
       * \brief parameter for the prox-function (r>0)
       */
      double r;

      /**
       *  \brief which solver is used (leads to different return values of function)
       */
      SolverType solverType;

      /**
       * \brief parameter to print information
       */
      unsigned int DEBUGLEVEL;

  };

  //TODO: LCPReformulationJacobian for NewtonMethod to avoid numerical Jacobian

}

#endif /*LINEAR_COMPLEMENTARITY_PROBLEM_H_*/
