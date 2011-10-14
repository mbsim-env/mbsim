/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef  _ANSATZ_FUNCTIONS_H_
#define  _ANSATZ_FUNCTIONS_H_

#include "fmatvec.h"

  /**
   * \brief a class of ansatz-functions for approximations with Galerkin-Method
   * \date 2009-07-22 check in (Markus Schneider, Thanks to Markus Friedrich)
   */
class ansatz_function {
  public:
    /** Return the matrix of the 0-derive of the basic function */
    fmatvec::SymMat MatIntWWT();

    /** Return the matrix of the 1-derive of the basic function */
    fmatvec::SymMat MatIntWSWST();

    /** Return the vector of w(x=0) */
    fmatvec::Vec VecW0();

    /** Return the vector of W(x=L) */
    fmatvec::Vec VecWL();

    /** Return the vector of int(W, x=0..L) */
    fmatvec::Vec VecIntW();

    /** Returns the value of "w(xi)", 0<=xi<=1 */
    fmatvec::Vec VecW(double xi);

    /** Returns the value of "D(w)(xi)", 0<=xi<=1 */
    fmatvec::Vec VecWS(double xi);

    /** Return the order of the system */
    int order();

    /** Return the dimension of the system */
    int dim();

    /** Constructor
      @param n Order
      @param l Lenght of the conduction */
    ansatz_function(int n, double l);

    /** Descructor */
    virtual ~ansatz_function();

  protected:
    /** Returns the value of "int(w[i]*w[j],x=0..L)" */
    virtual double Intwiwj(int i, int j)=0;

    /** Returns the value of "int(D(w[i])(x)*D(w[j])(x),x=0..L) */
    virtual double Intwsiwsj(int i, int j)=0;

    /** Returns the value of "w[i](x=0)" */
    virtual double wi0(int i)=0;

    /** Returns the value of "w[i](x=L)" */
    virtual double wiL(int i)=0;

    /** Returns the value of "int(w[i](x), x=0..L)" */
    virtual double Intwi(int i)=0;

    /** Returns the value of "w[i](xi)", 0<=xi<=1 */
    virtual double wi(int i, double xi)=0;

    /** Returns the value of "D(w[i])(xi)", 0<=xi<=1 */
    virtual double wis(int i, double xi)=0;

    /** Var for the order of the system */
    int Ord;

    /** Var for the dimensoin of the returned matrix */
    int Dim;

    /** Var for the lenght of the condunction */
    double L;
};


/** Approximate with the Galerkin-Methode using harmonic function */
class ansatz_function_harmonic : public ansatz_function {
  public:
    /** Constructor
      @param n Order
      @param l Lenght of the conduction */
    ansatz_function_harmonic(int n, double l);

  protected:
    /** Returns the value of "int(w[i]*w[j],x=0..L) */
    virtual double Intwiwj(int i, int j);

    /** Returns the value of "int(D(w[i])(x)*D(w[j])(x),x=0..L) */
    virtual double Intwsiwsj(int i, int j);

    /** Returns the value of "w[i](x=0)" */
    virtual double wi0(int i);

    /** Returns the value of "w[i](x=L)" */
    virtual double wiL(int i);

    /** Returns the value of "int(w[i](x), x=0..L)" */
    virtual double Intwi(int i);

    /** Returns the value of "w[i](xi)", 0<=xi<=1 */
    virtual double wi(int i, double xi);

    /** Returns the value of "D(w[i])(xi)", 0<=xi<=1 */
    virtual double wis(int i, double xi);
};


/** Approximate with the Galerkin-Methode using polynom function */
class ansatz_function_polynom : public ansatz_function {
  public:
    /** Constructor
      @param n Order
      @param l Lenght of the conduction */
    ansatz_function_polynom(int n, double l);

  protected:
    /** Returns the value of "int(w[i]*w[j],x=0..L) */
    virtual double Intwiwj(int i, int j);

    /** Returns the value of "int(D(w[i])(x)*D(w[j])(x),x=0..L) */
    virtual double Intwsiwsj(int i, int j);

    /** Returns the value of "w[i](x=0)" */
    virtual double wi0(int i);

    /** Returns the value of "w[i](x=L)" */
    virtual double wiL(int i);

    /** Returns the value of "int(w[i](x), x=0..L)" */
    virtual double Intwi(int i);

    /** Returns the value of "w[i](xi)", 0<=xi<=1 */
    virtual double wi(int i, double xi);

    /** Returns the value of "D(w[i])(xi)", 0<=xi<=1 */
    virtual double wis(int i, double xi);
};



/** Approximate with the Galerkin-Methode using B-Spline functions (Order 3) */
class ansatz_function_BSplineOrd3 : public ansatz_function {
  public:
    /** Constructor
      @param n Order
      @param l Lenght of the conduction */
    ansatz_function_BSplineOrd3(int n, double l);

  protected:
    /** Returns the value of "int(w[i]*w[j],x=0..L) */
    virtual double Intwiwj(int i, int j);

    /** Returns the value of "int(D(w[i])(x)*D(w[j])(x),x=0..L) */
    virtual double Intwsiwsj(int i, int j);

    /** Returns the value of "w[i](x=0)" */
    virtual double wi0(int i);

    /** Returns the value of "w[i](x=L)" */
    virtual double wiL(int i);

    /** Returns the value of "int(w[i](x), x=0..L)" */
    virtual double Intwi(int i);

    /** Returns the value of "w[i](xi)", 0<=xi<=1 */
    virtual double wi(int i, double xi);

    /** Returns the value of "D(w[i])(xi)", 0<=xi<=1 */
    virtual double wis(int i, double xi);
};


/** Approximate with the Galerkin-Methode using B-Spline functions (Order 4) */
class ansatz_function_BSplineOrd4 : public ansatz_function {
  public:
    /** Constructor
      @param n Order
      @param l Lenght of the conduction */
    ansatz_function_BSplineOrd4(int n, double l);

  protected:
    /** Returns the value of "int(w[i]*w[j],x=0..L) */
    virtual double Intwiwj(int i, int j);

    /** Returns the value of "int(D(w[i])(x)*D(w[j])(x),x=0..L) */
    virtual double Intwsiwsj(int i, int j);

    /** Returns the value of "w[i](x=0)" */
    virtual double wi0(int i);

    /** Returns the value of "w[i](x=L)" */
    virtual double wiL(int i);

    /** Returns the value of "int(w[i](x), x=0..L)" */
    virtual double Intwi(int i);

    /** Returns the value of "w[i](xi)", 0<=xi<=1 */
    virtual double wi(int i, double xi);

    /** Returns the value of "D(w[i])(xi)", 0<=xi<=1 */
    virtual double wis(int i, double xi);
};

#endif   /* ----- #ifndef _ANSATZ_FUNCTIONS_H_  ----- */

