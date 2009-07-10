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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef NONLINEAR_ALGEBRA_H_
#define NONLINEAR_ALGEBRA_H_

#include "mbsim/utils/function.h"
#include "fmatvec.h"

namespace MBSim {

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
      RegulaFalsi(Function<double,double> *f);

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
      Function<double,double> *func;

      /**
       * \brief maximum number of iterations, actual number of iterations, information about success (0 = ok, -1 = not converged, -2 = no root) 
       */
      int itmax, it, info;

      /** 
       * \brief tolerance
       */
      double tol;
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
      NewtonMethod(Function<double,double> *fct_, Function<double,double> *jac_=0);

      /* GETTER / SETTER */
      int getNumberOfIterations() const { return iter; }
      int getInfo() const { return info; }
      void setMaximumNumberOfIterations(int itmax_) { itmax = itmax_; }
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
      Function<double,double> *fct;

      /**
       * \brief Jacobian matrix
       */
      Function<double,double> *jac;

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
      MultiDimNewtonMethod(Function<fmatvec::Vec,fmatvec::Vec> *fct_, Function<fmatvec::SqrMat,fmatvec::Vec> *jac_=0);

      /* GETTER / SETTER */
      int getNumberOfIterations() const { return iter; }
      int getInfo() const { return info; }
      void setMaximumNumberOfIterations(int itmax_) { itmax = itmax_; }
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
      Function<fmatvec::Vec,fmatvec::Vec> *fct;

      /**
       * \brief Jacobian matrix
       */
      Function<fmatvec::SqrMat,fmatvec::Vec> *jac;

      /** 
       * \brief maximum number of iterations, actual number of iterations, maximum number of damping steps, information about success 
       */
      int itmax, iter, kmax, info;
      
      /** 
       * \brief tolerance
       */
      double tol;
  };

}

#endif /* NONLINEAR_ALGEBRA_H_ */

