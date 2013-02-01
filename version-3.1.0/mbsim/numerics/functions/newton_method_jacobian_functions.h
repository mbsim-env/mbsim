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

  /*!
   * \brief base class for square Jacobians used for the newton method
   */
  template<int size>
  class NJacobianFunction : public Function1<fmatvec::SquareMatrix<fmatvec::Fixed<size>, double >, fmatvec::Vector<fmatvec::Fixed<size>, double > > {
    public:
    /**
     * \brief constructor
     */
      NJacobianFunction();

    /*
     * \brief destructor
     */
    virtual ~NJacobianFunction() {
    }

    /* GETTER / SETTER*/
    virtual void setFunction(Function1<fmatvec::Vector<fmatvec::Fixed<size>, double >, fmatvec::Vector<fmatvec::Fixed<size>, double > > * function_) {
      function = function_;
    }
    /*****************/

    /*INHERITED INTERFACE*/
    virtual fmatvec::SquareMatrix<fmatvec::Fixed<size>, double > operator ()(const fmatvec::Vector<fmatvec::Fixed<size>, double > & x, const void* = NULL) = 0;
    /*********************/

    protected:
    Function1<fmatvec::Vector<fmatvec::Fixed<size>, double >, fmatvec::Vector<fmatvec::Fixed<size>, double> > * function;
  };

  /*!
   * \brief class to compute the Jacobian matrix for the newton method numerically
   */
  template<int size>
  class NumericalNJacobianFunction : public NJacobianFunction<size> {

      typedef fmatvec::Vector<fmatvec::Fixed<size>,double > vctr;
      typedef fmatvec::SquareMatrix<fmatvec::Fixed<size>,double > matr;

    public:
      /**
       * \brief constructor
       */
      NumericalNJacobianFunction();

      /*
       * \brief destructor
       */
      virtual ~NumericalNJacobianFunction() {
      }

      /*INHERITED INTERFACE*/
      virtual fmatvec::SquareMatrix<fmatvec::Fixed<size>, double > operator ()(const fmatvec::Vector<fmatvec::Fixed<size>, double> & x, const void* = NULL);


  };

  template<int size>
  NJacobianFunction<size>::NJacobianFunction() :
    function(0){
  }

  template <int size>
  NumericalNJacobianFunction<size>::NumericalNJacobianFunction() :
    NJacobianFunction<size>() {
  }

  template <int size>
  fmatvec::SquareMatrix<fmatvec::Fixed<size>,double > NumericalNJacobianFunction<size>::operator ()(const vctr & x, const void*) {
    matr J; // initialize size

    double dx, xj;
    vctr x2 = x;
    vctr f = (*NJacobianFunction<size>::function)(x2);
    vctr f2;

    for (int j = 0; j < size; j++) {
      xj = x2(j);

      dx = (epsroot() * 0.5);
      do {
        dx += dx;
      } while (fabs(xj + dx - x2(j)) < epsroot());

      x2(j) += dx;
      f2 = (*NJacobianFunction<size>::function)(x2);
      x2(j) = xj;
      vctr tmp = (f2 - f) / dx;
      for (int i = 0; i < size; i++)
        J(i,j) = tmp(i);
    }

    return J;
  }

//  /*!
//   * \brief class to compute a Jacobian matrix once at the beginning and then uses it over and over again
//   *
//   * \todo: implement
//   */
//  class ConstantNewtonJacobianFunction : public NewtonJacobianFunction {
//    public:
//      /**
//       * \brief constructor
//       */
//      ConstantNewtonJacobianFunction();
//
//      /*
//       * \brief destructor
//       */
//      virtual ~ConstantNewtonJacobianFunction() {
//      }
//
//      /*INHERITED INTERFACE*/
//      virtual fmatvec::SqrMat operator ()(const fmatvec::Vec & x, Function1<fmatvec::Vec, fmatvec::Vec> * function);
//
//  };
//
//  /*!
//   * \brief class use the identity matrix as Jacobian matrix in the newton method yields a fixpoint iteration
//   *
//   * \todo: implement
//   */
//  class FixpointNewtonJacobianFunction : public NewtonJacobianFunction {
//    public:
//      /**
//       * \brief constructor
//       */
//      FixpointNewtonJacobianFunction();
//
//      /*
//       * \brief destructor
//       */
//      virtual ~FixpointNewtonJacobianFunction() {
//      }
//
//      /*INHERITED INTERFACE*/
//      virtual fmatvec::SqrMat operator ()(const fmatvec::Vec & x, Function1<fmatvec::Vec, fmatvec::Vec> * function);
//  };
}
#endif // NUMERICSNEWTONMEHTODFUNCTION_H_
