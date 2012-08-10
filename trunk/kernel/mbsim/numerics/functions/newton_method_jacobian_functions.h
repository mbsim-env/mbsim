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

#ifndef NUMERICSNEWTONMEHTODFUNCTION_H_
#define NUMERICSNEWTONMEHTODFUNCTION_H_

#include <mbsim/utils/function.h>

#include <fmatvec.h>

namespace MBSim {

  /*!
   * \brief base class for square Jacobians used for the newton method
   */
  class NewtonJacobianFunction : public Function1<fmatvec::SqrMat, fmatvec::Vec> {
    public:
    /**
     * \brief constructor
     */
    NewtonJacobianFunction();

    /*
     * \brief destructor
     */
    virtual ~NewtonJacobianFunction() {
    }

    /* GETTER / SETTER*/
    virtual void setFunction(Function1<fmatvec::Vec, fmatvec::Vec> * function_) {
      function = function_;
    }
    /*****************/

    /*INHERITED INTERFACE*/
    virtual fmatvec::SqrMat operator ()(const fmatvec::Vec & x, const void* = NULL) = 0;
    /*********************/

    protected:
    Function1<fmatvec::Vec, fmatvec::Vec> * function;
  };

  /*!
   * \brief class to compute the Jacobian matrix for the newton method numerically
   */
  class NumericalNewtonJacobianFunction : public NewtonJacobianFunction {
    public:
      /**
       * \brief constructor
       */
      NumericalNewtonJacobianFunction();

      /*
       * \brief destructor
       */
      virtual ~NumericalNewtonJacobianFunction() {
      }

      /*INHERITED INTERFACE*/
      virtual fmatvec::SqrMat operator ()(const fmatvec::Vec & x, const void* = NULL);


  };

  /*!
   * \brief class to compute a Jacobian matrix once at the beginning and then uses it over and over again
   *
   * \todo: implement
   */
  class ConstantNewtonJacobianFunction : public NewtonJacobianFunction {
    public:
      /**
       * \brief constructor
       */
      ConstantNewtonJacobianFunction();

      /*
       * \brief destructor
       */
      virtual ~ConstantNewtonJacobianFunction() {
      }

      /*INHERITED INTERFACE*/
      virtual fmatvec::SqrMat operator ()(const fmatvec::Vec & x, Function1<fmatvec::Vec, fmatvec::Vec> * function);

  };

  /*!
   * \brief class use the identity matrix as Jacobian matrix in the newton method yields a fixpoint iteration
   *
   * \todo: implement
   */
  class FixpointNewtonJacobianFunction : public NewtonJacobianFunction {
    public:
      /**
       * \brief constructor
       */
      FixpointNewtonJacobianFunction();

      /*
       * \brief destructor
       */
      virtual ~FixpointNewtonJacobianFunction() {
      }

      /*INHERITED INTERFACE*/
      virtual fmatvec::SqrMat operator ()(const fmatvec::Vec & x, Function1<fmatvec::Vec, fmatvec::Vec> * function);
  };
}
#endif // NUMERICSNEWTONMEHTODFUNCTION_H_
