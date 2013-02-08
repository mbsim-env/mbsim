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

#ifndef NUMERICSLCPREFORMULATEDFUNCTION_H_
#define NUMERICSLCPREFORMULATEDFUNCTION_H_

#include <mbsim/utils/function.h>

#include <mbsim/numerics/functions/newton_method_jacobian_functions.h>

namespace MBSim {

  class LCPReformulationFunction : public Function1<fmatvec::Vec, fmatvec::Vec> {
    public:
      /*!
       * \brief standard constructor
       */
      LCPReformulationFunction(const double &r_ = 10) :
          dimension(0), r(r_), DEBUGLEVEL(0) {
      }

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
      void setSystem(const fmatvec::SqrMat & M_, const fmatvec::Vec & q_) {
        M = M_;
        q = q_;
        assert(M.rows() == q.rows());
        dimension = q.rows();
      }
      fmatvec::SqrMat getM(void) {
        return M;
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
      int dimension;

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
      /*!
       * \brief standard constructor
       */
      LCPNewtonReformulationFunction() :
          LCPReformulationFunction() {
      }

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
      /*!
       * \brief standard constructor
       */
      LCPFixpointReformulationFunction() :
          LCPReformulationFunction() {
      }

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

  class LinearComplementarityJacobianFunction : public NewtonJacobianFunction<fmatvec::Ref, double> {
    public:
      /**
       * \brief constructor
       */
      LinearComplementarityJacobianFunction();

      /*
       * \brief destructor
       */
      virtual ~LinearComplementarityJacobianFunction();

      virtual void operator ()(const fmatvec::Vec & x, fmatvec::SqrMat & J, const void* = NULL);

      virtual void setFunction(Function1<fmatvec::Vec, fmatvec::Vec> * function_);

      void updateJacobian(const fmatvec::Vec & x, fmatvec::SqrMat & J);

    protected:
      /**
       * \brief Jacobian function (stays saved to save computation time, as some parts stay constant)
       */
      fmatvec::SqrMat J;

  };
}
#endif //NUMERICSLCPREFORMULATEDFUNCTION_H_
