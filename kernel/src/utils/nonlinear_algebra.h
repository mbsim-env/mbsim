/* Copyright (C) 2004-2006  Martin Förg
 
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
 *   mfoerg@users.berlios.de
 *
 */

#ifndef NONLINEAR_ALGEBRA_H_
#define NONLINEAR_ALGEBRA_H_

#include "function.h"
#include "fmatvec.h"

using namespace fmatvec;

namespace MBSim {

  /*! Regular Falsi for one-dimensional root-finding */
  class RegulaFalsi {
    private:
      Function<double,double> *func;
      int itmax,it,info;
      double tol;
    public:
    
      RegulaFalsi(Function<double,double> *f);
      int getIter() const {return it;};
      int getInfo() const {return info;};
      void setMaxIter(int itmax_) {itmax = itmax_;}
      void setTol(double tol_) {tol = tol_;}
      double slv(double a, double b);
  };

  /*! Newton method for one-dimensional root-finding */
  class NewtonMethod {
    private:
      Function<double,double> *fct;
      Function<double,double> *jac;

      int itmax, iter, prim, kmax, rc, info;
      double tol;
    public:
    
      NewtonMethod(Function<double,double> *fct_, Function<double,double> *jac_=0);
      int getIter() const {return iter;}
      int getInfo() const {return info;}
      void setMaxIter(int itmax_) {itmax = itmax_;}
      void setTol(double tol_) {tol = tol_;}
      double slv(const double &x);
      int nit() const {return iter;};
  };

  /*! Newton method for multi-dimensional root-finding */
  class MultiDimNewtonMethod {
    private:
      Function<Vec,Vec> *fct;
      Function<SqrMat,Vec> *jac;

      int itmax, iter, prim, kmax, rc, info;
      double tol;
      
    public:
      MultiDimNewtonMethod(Function<Vec,Vec> *fct_, Function<SqrMat,Vec> *jac_=0);
      int getIter() const {return iter;}
      int getInfo() const {return info;}
      void setMaxIter(int itmax_) {itmax = itmax_;}
      void setTol(double tol_) {tol = tol_;}
      Vec slv(const Vec &x);
      int nit() const {return iter;};
  };

}

#endif
