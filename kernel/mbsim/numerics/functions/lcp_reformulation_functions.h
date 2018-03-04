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

#include <mbsim/functions/function.h>

#include <mbsim/numerics/functions/newton_method_jacobian_functions.h>

namespace MBSim {

  class LCPReformulationFunction : public Function<fmatvec::Vec(fmatvec::Vec)> {
    public:
      /*!
       * \brief standard constructor
       */
      LCPReformulationFunction(const double &r_ = 10) :
          r(r_) {
      }

      /**
       * \brief constructor
       */
      LCPReformulationFunction(const fmatvec::Vec &q_, const fmatvec::SqrMat &M_, const double &r_ = 10);

      /*
       * \brief destructor
       */
      ~LCPReformulationFunction() override;

      /* INHERITED INTERFACE */
      /**
       * \param q: solution vector with
       *           first entries: w, last entries: z
       */
      fmatvec::Vec operator()(const fmatvec::Vec &q) override = 0;
      /***************************************************/

      /**GETTER / SETTER*/
      fmatvec::Vec getq() {
        return q;
      }
      void setSystem(const fmatvec::SqrMat & M_, const fmatvec::Vec & q_) {
        M.resize() = M_;
        q.resize() = q_;
        assert(M.rows() == q.rows());
        NumberOfContacts = q.rows();
      }
      fmatvec::SqrMat getM() {
        return M;
      }
      double getr() {
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
       */
      LCPNewtonReformulationFunction(const fmatvec::Vec &q_, const fmatvec::SqrMat &M_, const double &r_ = 10);

      /**
       * \brief destructor
       */
      ~LCPNewtonReformulationFunction() override;

      /* INHERITED INTERFACE */
      fmatvec::Vec operator()(const fmatvec::Vec &q) override;
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
       */
      LCPFixpointReformulationFunction(const fmatvec::Vec &q_, const fmatvec::SqrMat &M_, const double &r_ = 10);

      /**
       * \brief destructor
       */
      ~LCPFixpointReformulationFunction() override;

      /* INHERITED INTERFACE */
      fmatvec::Vec operator()(const fmatvec::Vec &q) override;
      /***************************************************/
  };

  class LinearComplementarityJacobianFunction : public NewtonJacobianFunction {
    public:
      /**
       * \brief constructor
       */
      LinearComplementarityJacobianFunction();

      /*
       * \brief destructor
       */
      ~LinearComplementarityJacobianFunction() override;

      fmatvec::SqrMat operator ()(const fmatvec::Vec & x) override;

      void setFunction(Function<fmatvec::Vec(fmatvec::Vec)> *function_) override;

      void updateJacobian(const fmatvec::Vec & x);

    protected:
      /**
       * \brief Jacobian function (stays saved to save computation time, as some parts stay constant)
       */
      fmatvec::SqrMat J;

  };
}
#endif //NUMERICSLCPREFORMULATEDFUNCTION_H_
