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

#include <config.h>
#include "nonlinear_algebra.h"
#include "eps.h"
#include <cmath>

namespace MBSim {

  RegulaFalsi::RegulaFalsi(Function<double,double> *f) : func(f), itmax(10000), tol(1e-10) {
  }

  double RegulaFalsi::slv(double a, double b) {

    double u;
    double fa, fb, fu;

    for(it=0; it<itmax; it++) {
      fa = (*func)(a);
      fb = (*func)(b);
      if(fa*fb > 0) {
	info = -2;
      }
      u = a - fa * (b-a)/(fb-fa);
      if(u<a)
	return u;
      else if(u>b)
	return u;
      fu = (*func)(u);

      if (fu*fa < 0) {
	b = u;
      } else {
	a = u;
      }
      if (fabs(fu) < tol) {
	info = 0;
	return u;
      }
    }
    info = -1;
    return u; 
  }

  NewtonMethod::NewtonMethod(Function<double,double> *fct_, Function<double,double> *jac_) : fct(fct_), jac(jac_), itmax(300), iter(0), prim(0), kmax(100), rc(0), tol(1e-10) {
  }

  double NewtonMethod::slv(const double &x0) {

    iter=0;
    double x = x0, xold;
    double nrmf;

    double f = (*fct)(x);
    double nrmf0 = fabs(f);
    double J;
    for (iter=1; iter <= itmax; iter++) {

      if (nrmf0 <= tol) {
	info = 0;
	return x0;
      }

      if(jac)
	J = (*jac)(x);
      else {
	double delta = epsroot();
	//Prüfen: sollte unnötig sein
	//double f_old = (*fct)(x);
	double f_new;
	double xtmp = x;
	x += delta;
	f_new = (*fct)(x);
	J = (f_new - f)/delta; // vorher f=f_old
	x = xtmp;
      }
      double dx = 0;
      if(J*J > 1.e-15)
	dx = f/J;
      else {
	info = -1;
	return rc;
      }

      double alpha = 1;
      xold = x;
      for (int k=0; k<kmax; k++) {
	x = xold - alpha*dx;
	f = (*fct)(x);
	nrmf = fabs(f);
	if(nrmf < nrmf0)
	  break;
	alpha *= 0.5;
      }
      nrmf0 = nrmf;
      if (nrmf <= tol) {
	info = 0;
	return x;
      }
    }
    info = -1;

    return rc;
  }

  MultiDimNewtonMethod::MultiDimNewtonMethod(Function<Vec,Vec> *fct_, Function<SqrMat,Vec> *jac_) : fct(fct_), jac(jac_), itmax(300), iter(0), prim(0), kmax(100), rc(0), tol(1e-10) {
  }

  Vec MultiDimNewtonMethod::slv(const Vec &x0) {

    iter=0;
    Vec x, xold;
    x = x0;
    double nrmf;

    Vec f = (*fct)(x);
    double nrmf0 = nrmInf(f);
    SqrMat J;
    for (iter=1; iter <= itmax; iter++) {

      if (nrmf0 <= tol) {
	info = 0;
	return x0;
      }

      if(jac)
	J = (*jac)(x);
      else {
	J = SqrMat(x.size()); // initialise size 
	double dx, xj;
	Vec f2;

	for(int j=0; j<x.size(); j++) {
	  xj = x(j);

	  dx = (epsroot() * 0.5);
	  do {                   
	    dx += dx;
	  } while (xj + dx == x(j));

	  x(j)+=dx;
	  f2 = (*fct)(x);
	  x(j)=xj;
	  J.col(j) = (f2-f)/dx;
	}
      }

      Vec dx = slvLU(J,f);

      double alpha = 1;
      xold = x;
      for (int k=0; k<kmax; k++) {
	x = xold - alpha*dx;
	f = (*fct)(x);
	nrmf = nrmInf(f);
	if(nrmf < nrmf0)
	  break;
	alpha *= 0.5;
      }
      nrmf0 = nrmf;
      if (nrmf <= tol) {
	info = 0;
	return x;
      }
    }
    info = -1;

    return rc;
  }

}
