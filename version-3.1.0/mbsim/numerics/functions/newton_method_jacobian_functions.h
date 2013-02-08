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
  template <class VecType, class AT>
  class NewtonJacobianFunction {

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
      virtual void setFunction(Function1<fmatvec::Vector<VecType, AT> , fmatvec::Vector<VecType, AT>  > * function_) {
        function = function_;
      }
      /*****************/

      /*INHERITED INTERFACE*/
      virtual void operator ()(const fmatvec::Vector<VecType, AT>  & x, fmatvec::SquareMatrix<VecType, AT> & J, const void* = NULL) = 0;
      /*********************/

    protected:
      Function1<fmatvec::Vector<VecType, AT> , fmatvec::Vector<VecType, AT> > * function;
  };

  /*!
   * \brief class to compute the Jacobian matrix for the newton method numerically
   */
  template <class VecType, class AT>
  class NumericalNewtonJacobianFunction : public NewtonJacobianFunction<VecType, AT> {
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
      virtual void operator ()(const fmatvec::Vector<VecType, AT> & x, fmatvec::SquareMatrix<VecType, AT> & J, const void* = NULL);

  };

  /*!
   * \brief class to compute a Jacobian matrix once at the beginning and then uses it over and over again
   *
   * \todo: implement
   */
  template <class VecType, class AT>
  class ConstantNewtonJacobianFunction : public NewtonJacobianFunction<VecType, AT> {
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
      virtual fmatvec::SquareMatrix<VecType, AT> operator ()(const fmatvec::Vec & x, Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > * function);

  };

  /*!
   * \brief class use the identity matrix as Jacobian matrix in the newton method yields a fixpoint iteration
   *
   * \todo: implement
   */
  template <class VecType, class AT>
  class FixpointNewtonJacobianFunction : public NewtonJacobianFunction<VecType, AT> {
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
      virtual fmatvec::SquareMatrix<VecType, AT> operator ()(const fmatvec::Vector<VecType, AT> & x, Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > * function);
  };

  template <class VecType, class AT>
  NewtonJacobianFunction<VecType, AT>::NewtonJacobianFunction() :
      function(0) {
  }

  template <class VecType, class AT>
  NumericalNewtonJacobianFunction<VecType, AT>::NumericalNewtonJacobianFunction() :
      NewtonJacobianFunction<VecType, AT>() {
  }

  template <class VecType, class AT>
  void NumericalNewtonJacobianFunction<VecType, AT>::operator ()(const fmatvec::Vector<VecType, AT> & x, fmatvec::SquareMatrix<VecType, AT> & J, const void*) {

    double dx, xj;
    fmatvec::Vector<VecType, AT> x2 = x;
    fmatvec::Vector<VecType, AT> f = (*this->function)(x2);
    fmatvec::Vector<VecType, AT> f2(x2.size(), fmatvec::NONINIT);

    for (int j = 0; j < x2.size(); j++) {
      xj = x2(j);

      dx = (epsroot() * 0.5);
      do {
        dx += dx;
      } while (fabs(xj + dx - x2(j)) < epsroot());

      x2(j) += dx;
      f2 = (*this->function)(x2);
      x2(j) = xj;
      J.set(j, (f2 - f) / dx);
    }
  }

}
#endif // NUMERICSNEWTONMEHTODFUNCTION_H_
